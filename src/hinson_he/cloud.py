"""Convert a Hinson revolution into a Viam point cloud and a preview image."""

from __future__ import annotations

import math
import struct

from PIL import Image, ImageDraw

from hinson_he.protocol import FOV_DEGREES, MAX_DISTANCE_MM, SCAN_START_DEG, Scan

PCD_MIME = "pointcloud/pcd"


def sensor_angle_deg(index: int, count: int) -> float:
    if count <= 0:
        return SCAN_START_DEG
    return SCAN_START_DEG + index * (FOV_DEGREES / count)


def project_points(
    scan: Scan,
    *,
    forward_axis: str,
    angle_offset_deg: float,
    start_angle_deg: float,
    end_angle_deg: float,
    min_range_mm: float,
    max_range_mm: float,
) -> list[tuple[float, float, float, int]]:
    """Return x, y, z millimeters and intensity.

    Sensor-cover angle 0 is the mark on the lidar lid. The published ROS driver
    places that mark 180 degrees from +X, or 90 degrees from +Y when
    ``forward_axis`` is ``y``. Coordinates are millimeters, matching the Viam
    RPLidar camera.
    """
    count = len(scan.distances_mm)
    axis_offset = -180.0 if forward_axis == "x" else -90.0
    points: list[tuple[float, float, float, int]] = []
    intensities = scan.intensities or [0] * count
    for index, distance in enumerate(scan.distances_mm):
        if distance < min_range_mm or distance > max_range_mm or distance > MAX_DISTANCE_MM:
            continue
        angle = sensor_angle_deg(index, count)
        if angle < start_angle_deg or angle > end_angle_deg:
            continue
        theta = math.radians(angle + axis_offset + angle_offset_deg)
        intensity = intensities[index] if index < len(intensities) else 0
        points.append((distance * math.cos(theta), distance * math.sin(theta), 0.0, intensity))
    return points


def encode_pcd(points: list[tuple[float, float, float, int]]) -> bytes:
    """Encode points as binary PCD.

    ``points`` are millimeters. Viam's PCD reader treats file coordinates as
    meters and converts them back to millimeters, so the file stores meters.
    """
    header = (
        "# .PCD v.7 - Point Cloud Data file format\n"
        "VERSION .7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    payload = bytearray()
    for x_mm, y_mm, z_mm, _intensity in points:
        payload += struct.pack("<fff", x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)
    return header + payload


def render_preview(
    points: list[tuple[float, float, float, int]],
    *,
    max_range_mm: float,
    size: int = 480,
) -> bytes:
    image = Image.new("RGB", (size, size), (16, 18, 22))
    draw = ImageDraw.Draw(image)
    center = size / 2
    margin = 24
    scale = (center - margin) / max(max_range_mm, 1.0)
    draw.line((center, margin, center, size - margin), fill=(40, 44, 52))
    draw.line((margin, center, size - margin, center), fill=(40, 44, 52))
    draw.ellipse((center - 3, center - 3, center + 3, center + 3), fill=(220, 224, 230))
    peak = max((point[3] for point in points), default=1) or 1
    for x, y, _z, intensity in points:
        px = center + x * scale
        py = center - y * scale
        shade = int(80 + 175 * min(intensity, peak) / peak)
        draw.point((px, py), fill=(shade, shade, 210))
    return _png_bytes(image)


def _png_bytes(image: Image.Image) -> bytes:
    from io import BytesIO

    buffer = BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()
