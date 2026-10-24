"""Viam camera model for a Hinson HE lidar."""

from __future__ import annotations

import threading
from typing import Any, Mapping, Sequence

from viam.components.camera import Camera
from viam.media.video import CameraMimeType, NamedImage
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName, ResponseMetadata
from viam.proto.component.camera import GetPropertiesResponse
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.utils import ValueTypes, struct_to_dict
from google.protobuf.timestamp_pb2 import Timestamp

from hinson_he.client import HinsonClient, LidarConfig
from hinson_he.cloud import PCD_MIME, encode_pcd, project_points, render_preview
from hinson_he.protocol import ANGLE_INCREMENTS

MODEL_NAME = "viam-labs:camera:hinson-he"


class HinsonHE(Camera, EasyResource):
    MODEL = MODEL_NAME

    def __init__(self, name: str):
        super().__init__(name)
        self._lock = threading.Lock()
        self._client: HinsonClient | None = None
        self._view = _default_view()

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        camera = super().new(config, dependencies)
        camera.reconfigure(config, dependencies)
        return camera

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> tuple[Sequence[str], Sequence[str]]:
        parse_config(struct_to_dict(config.attributes))
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        lidar_config, view = parse_config(struct_to_dict(config.attributes))
        with self._lock:
            previous = self._client
            self._client = None
        if previous is not None:
            previous.close()
        client = HinsonClient(lidar_config, self.logger)
        with self._lock:
            self._client = client
            self._view = view

    async def get_point_cloud(
        self,
        *,
        extra: dict[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ) -> tuple[bytes, str]:
        scan, view = self._fresh_scan()
        points = project_points(scan, **view)
        return encode_pcd(points), PCD_MIME

    async def get_images(
        self,
        *,
        filter_source_names: Sequence[str] | None = None,
        extra: dict[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ) -> tuple[Sequence[NamedImage], ResponseMetadata]:
        if filter_source_names and self.name not in filter_source_names:
            return [], _captured_now()
        scan, view = self._fresh_scan()
        points = project_points(scan, **view)
        png = render_preview(points, max_range_mm=view["max_range_mm"])
        return [NamedImage(self.name, png, CameraMimeType.PNG)], _captured_now()

    async def get_properties(
        self, *, timeout: float | None = None, **kwargs
    ) -> Camera.Properties:
        with self._lock:
            rate = (
                float(self._client.config.spin_frequency_hz)
                if self._client is not None
                else 0.0
            )
        return GetPropertiesResponse(
            supports_pcd=True,
            frame_rate=rate,
            mime_types=[CameraMimeType.PNG, CameraMimeType.PCD],
        )

    async def get_geometries(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None
    ) -> Sequence[Geometry]:
        return []

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: float | None = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        client = self._require_client()
        if "channel" in command and command["channel"] is not None:
            client.set_channel(int(command["channel"]))  # type: ignore[arg-type]
        status = client.status()
        status["command"] = dict(command)
        return status

    async def get_status(
        self, *, timeout: float | None = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        return self._require_client().status()

    async def close(self):
        with self._lock:
            client = self._client
            self._client = None
        if client is not None:
            client.close()

    def _fresh_scan(self):
        client = self._require_client()
        with self._lock:
            view = dict(self._view)
        scan = client.latest_scan()
        if scan is None:
            status = client.status()
            detail = status["error"] or "no scan yet"
            raise ConnectionError(f"Hinson HE lidar has no recent scan ({detail})")
        return scan, view

    def _require_client(self) -> HinsonClient:
        with self._lock:
            if self._client is None:
                raise ConnectionError("Hinson HE lidar is not configured")
            return self._client


def _default_view() -> dict[str, Any]:
    return {
        "forward_axis": "y",
        "angle_offset_deg": 0.0,
        "start_angle_deg": 0.0,
        "end_angle_deg": 360.0,
        "min_range_mm": 50.0,
        "max_range_mm": 50000.0,
    }


def parse_config(attributes: Mapping[str, Any]) -> tuple[LidarConfig, dict[str, Any]]:
    address = _string(attributes, "laser_ip", "192.168.1.88")
    if not address:
        raise ValueError("laser_ip must be a non-empty address")
    port = _int(attributes, "laser_port", 8080)
    if port < 1 or port > 65535:
        raise ValueError("laser_port must be between 1 and 65535")
    change_param = _bool(attributes, "change_param", False)
    spin = _int(attributes, "spin_frequency_hz", 10)
    increment = _string(attributes, "angle_increment", "0.050")
    noise = _int(attributes, "noise_filter_level", 1)
    if change_param:
        if spin not in (10, 20):
            raise ValueError("spin_frequency_hz must be 10 or 20")
        if increment not in ANGLE_INCREMENTS:
            raise ValueError(
                f"angle_increment must be one of {', '.join(ANGLE_INCREMENTS)}"
            )
        if noise < 0 or noise > 3:
            raise ValueError("noise_filter_level must be from 0 to 3")
    forward = _string(attributes, "forward_axis", "y").lower()
    if forward not in ("x", "y"):
        raise ValueError("forward_axis must be 'x' or 'y'")
    start_angle = _float(attributes, "start_angle_deg", 0.0)
    end_angle = _float(attributes, "end_angle_deg", 360.0)
    if start_angle < 0 or end_angle > 360 or start_angle > end_angle:
        raise ValueError(
            "start_angle_deg and end_angle_deg must satisfy 0 <= start <= end <= 360"
        )
    min_range = _float(attributes, "min_range_mm", 50.0)
    max_range = _float(attributes, "max_range_mm", 50000.0)
    if min_range < 0 or max_range <= min_range:
        raise ValueError("min_range_mm must be >= 0 and less than max_range_mm")
    shadows_level = _int(attributes, "shadows_filter_level", 0)
    if shadows_level < -1 or shadows_level > 3:
        raise ValueError("shadows_filter_level must be from -1 to 3")
    lidar = LidarConfig(
        address=address,
        port=port,
        use_udp=_bool(attributes, "use_udp", False),
        change_param=change_param,
        spin_frequency_hz=spin,
        angle_increment=increment,
        noise_filter_level=noise,
        shadows_filter_level=shadows_level,
        shadows_filter_min_angle_deg=_float(
            attributes, "shadows_filter_min_angle_deg", 5.0
        ),
        shadows_filter_max_angle_deg=_float(
            attributes, "shadows_filter_max_angle_deg", 175.0
        ),
        shadows_filter_window=_int(attributes, "shadows_filter_window", 2),
        shadows_filter_traverse_step=max(
            _int(attributes, "shadows_filter_traverse_step", 1), 1
        ),
        disturb_filter_enable=_bool(attributes, "disturb_filter_enable", False),
        disturb_filter_threshold_mm=_int(attributes, "disturb_filter_threshold_mm", 80),
        disturb_filter_point_num=max(
            _int(attributes, "disturb_filter_point_num", 8), 1
        ),
        block_enable=_bool(attributes, "block_enable", False),
        area_channel=_int(attributes, "area_channel", 0),
    )
    view = {
        "forward_axis": forward,
        "angle_offset_deg": _float(attributes, "angle_offset_deg", 0.0),
        "start_angle_deg": start_angle,
        "end_angle_deg": end_angle,
        "min_range_mm": min_range,
        "max_range_mm": max_range,
    }
    return lidar, view


def _captured_now() -> ResponseMetadata:
    timestamp = Timestamp()
    timestamp.GetCurrentTime()
    return ResponseMetadata(captured_at=timestamp)


def _string(attributes: Mapping[str, Any], key: str, default: str) -> str:
    value = attributes.get(key, default)
    if not isinstance(value, str):
        raise ValueError(f"{key} must be a string")
    return value


def _bool(attributes: Mapping[str, Any], key: str, default: bool) -> bool:
    value = attributes.get(key, default)
    if not isinstance(value, bool):
        raise ValueError(f"{key} must be a boolean")
    return value


def _int(attributes: Mapping[str, Any], key: str, default: int) -> int:
    value = attributes.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    return int(value)


def _float(attributes: Mapping[str, Any], key: str, default: float) -> float:
    value = attributes.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    return float(value)
