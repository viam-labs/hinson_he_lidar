"""TCP/UDP client that keeps the newest Hinson HE revolution."""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass

from hinson_he.protocol import (
    START_CAPTURE,
    AreaStatus,
    Scan,
    ScanParser,
    build_area_command,
    build_param_command,
    disturb_filter,
    shadows_filter,
)


@dataclass
class LidarConfig:
    address: str = "192.168.1.88"
    port: int = 8080
    use_udp: bool = False
    change_param: bool = False
    spin_frequency_hz: int = 10
    angle_increment: str = "0.050"
    noise_filter_level: int = 1
    shadows_filter_level: int = 0
    shadows_filter_min_angle_deg: float = 5.0
    shadows_filter_max_angle_deg: float = 175.0
    shadows_filter_window: int = 2
    shadows_filter_traverse_step: int = 1
    disturb_filter_enable: bool = False
    disturb_filter_threshold_mm: int = 80
    disturb_filter_point_num: int = 8
    block_enable: bool = False
    area_channel: int = 0


class HinsonClient:
    def __init__(self, config: LidarConfig, logger) -> None:
        self.config = config
        self._logger = logger
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._scan: Scan | None = None
        self._scan_monotonic = 0.0
        self._connected = False
        self._error = ""
        self._area = AreaStatus()
        self._channel = config.area_channel
        self._socket: socket.socket | None = None
        self._thread = threading.Thread(target=self._run, name="hinson-he", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._close_socket()
        self._thread.join(timeout=3)

    def latest_scan(self, max_age_sec: float = 2.0) -> Scan | None:
        with self._lock:
            if self._scan is None:
                return None
            if time.monotonic() - self._scan_monotonic > max_age_sec:
                return None
            return self._scan

    def set_channel(self, channel: int) -> None:
        with self._lock:
            self._channel = int(channel)

    def status(self) -> dict:
        with self._lock:
            age = None
            points = 0
            if self._scan is not None:
                age = round(time.monotonic() - self._scan_monotonic, 3)
                points = len(self._scan.distances_mm)
            return {
                "connected": self._connected,
                "error": self._error,
                "address": self.config.address,
                "port": self.config.port,
                "point_count": points,
                "scan_age_sec": age if age is not None else -1.0,
                "area1": self._area.area1,
                "area2": self._area.area2,
                "area3": self._area.area3,
                "area_channel": self._area.channel,
                "requested_channel": self._channel,
            }

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._session()
            except Exception as exc:
                if self._stop.is_set():
                    return
                with self._lock:
                    self._connected = False
                    self._error = str(exc)
                self._logger.warning("Hinson HE connection failed: %s", exc)
                self._stop.wait(1.0)

    def _session(self) -> None:
        parser = ScanParser()
        sock = self._open_socket()
        with self._lock:
            self._socket = sock
            self._connected = True
            self._error = ""
            channel = self._channel
        self._logger.info(
            "connected to Hinson HE at %s:%s (%s)",
            self.config.address,
            self.config.port,
            "udp" if self.config.use_udp else "tcp",
        )
        try:
            if self.config.change_param:
                sock.sendall(
                    build_param_command(
                        spin_frequency_hz=self.config.spin_frequency_hz,
                        angle_increment=self.config.angle_increment,
                        noise_filter_level=self.config.noise_filter_level,
                    )
                )
            sock.sendall(START_CAPTURE)
            next_area = time.monotonic()
            while not self._stop.is_set():
                now = time.monotonic()
                if self.config.block_enable and now >= next_area:
                    with self._lock:
                        channel = self._channel
                    sock.sendall(build_area_command(channel=channel))
                    next_area = now + 0.1
                try:
                    packet = sock.recv(4096)
                except TimeoutError:
                    continue
                if not packet:
                    raise ConnectionError("lidar closed the connection")
                parsed = parser.feed(packet)
                if parsed.area is not None:
                    with self._lock:
                        self._area = parsed.area
                for scan in parsed.scans:
                    scan.distances_mm = shadows_filter(
                        scan.distances_mm,
                        level=self.config.shadows_filter_level,
                        min_angle_deg=self.config.shadows_filter_min_angle_deg,
                        max_angle_deg=self.config.shadows_filter_max_angle_deg,
                        window=self.config.shadows_filter_window,
                        traverse_step=self.config.shadows_filter_traverse_step,
                    )
                    scan.distances_mm = disturb_filter(
                        scan.distances_mm,
                        enabled=self.config.disturb_filter_enable,
                        threshold_mm=self.config.disturb_filter_threshold_mm,
                        point_num=self.config.disturb_filter_point_num,
                    )
                    with self._lock:
                        self._scan = scan
                        self._scan_monotonic = time.monotonic()
                        self._connected = True
                        self._error = ""
        finally:
            self._close_socket()
            with self._lock:
                self._connected = False

    def _open_socket(self) -> socket.socket:
        kind = socket.SOCK_DGRAM if self.config.use_udp else socket.SOCK_STREAM
        sock = socket.socket(socket.AF_INET, kind)
        sock.settimeout(0.5)
        if not self.config.use_udp:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.connect((self.config.address, self.config.port))
        return sock

    def _close_socket(self) -> None:
        with self._lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
