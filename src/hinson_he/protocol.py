"""Hinson HE scan protocol.

The frame layout and filters follow the Hinson ROS driver (BSD-3-Clause,
Copyright (c) 2022, HINSON). This file is a standalone implementation of that
on-wire behavior: range packets begin with ``HISN``, distances are millimeters,
and one revolution runs from 20 degrees to 340 degrees on the sensor cover.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

START_CAPTURE = bytes([0x52, 0x41, 0x75, 0x74, 0x6F, 0x01, 0x87, 0x80])
RANGE_MAGIC = b"HISN"
AREA_MAGIC = b"WSimu"
HEADER_SIZE = 16
AREA_PACKET_SIZE = 13
MAX_DISTANCE_MM = 50000
MAX_INTENSITY = 30000
FOV_DEGREES = 320.0
SCAN_START_DEG = 20
SCAN_END_DEG = 340
ANGLE_INCREMENTS = ("0.025", "0.050", "0.100", "0.250", "0.500")


@dataclass
class Scan:
    distances_mm: list[int]
    intensities: list[int]
    time_increment_us: int = 0


@dataclass
class AreaStatus:
    channel: int = -1
    output: int = 0

    @property
    def area1(self) -> bool:
        return bool(self.output & 0x01)

    @property
    def area2(self) -> bool:
        return bool(self.output & 0x02)

    @property
    def area3(self) -> bool:
        return bool(self.output & 0x04)


@dataclass
class ParseResult:
    scans: list[Scan] = field(default_factory=list)
    area: AreaStatus | None = None


def crc16(data: bytes) -> int:
    """Modbus CRC-16, polynomial 0xA001, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_param_command(
    *,
    run: bool = True,
    spin_frequency_hz: int = 10,
    angle_increment: str = "0.050",
    noise_filter_level: int = 1,
) -> bytes:
    """Twelve-byte scan-parameter command.

    Matches the vendor command layout. A resolution of 0.025 forces the
    frequency byte back to 10 Hz, which is what the vendor driver sends.
    """
    command = bytearray(12)
    command[0:5] = b"SCtrl"
    command[5] = 0x00 if run else 0x01
    command[6] = 0x00
    command[7] = 0x01 if spin_frequency_hz == 20 else 0x00
    resolutions = {
        "0.025": 0x00,
        "0.050": 0x01,
        "0.100": 0x02,
        "0.250": 0x03,
        "0.500": 0x04,
    }
    if angle_increment not in resolutions:
        command[7] = 0x00
        command[8] = 0x00
    else:
        command[8] = resolutions[angle_increment]
        if angle_increment == "0.025":
            command[7] = 0x00
    if noise_filter_level < 0 or noise_filter_level > 3:
        noise_filter_level = 1
    command[9] = noise_filter_level
    crc = crc16(command[:10])
    command[10] = crc & 0xFF
    command[11] = (crc >> 8) & 0xFF
    return bytes(command)


def build_area_command(
    *, channel: int = 0, mode: int = 1, channel_group: int = 2
) -> bytes:
    """Eighteen-byte obstacle-area request."""
    if channel < 0 or channel > 63:
        channel = 0
    if channel_group < 0 or channel_group > 4:
        channel_group = 0
    command = bytearray(18)
    command[0:5] = b"WSimu"
    command[5] = mode & 0xFF
    command[6] = channel & 0xFF
    command[11] = (channel_group >> 8) & 0xFF
    command[12] = channel_group & 0xFF
    crc = crc16(command[:16])
    command[16] = crc & 0xFF
    command[17] = (crc >> 8) & 0xFF
    return bytes(command)


def shadows_filter(
    distances_mm: list[int],
    *,
    level: int,
    min_angle_deg: float,
    max_angle_deg: float,
    window: int,
    traverse_step: int,
) -> list[int]:
    """Drop the longer return when neighboring beams form a trailing-edge spike."""
    if level == 0 or not distances_mm:
        return distances_mm
    scan_num = len(distances_mm)
    if level == 1:
        min_angle, max_angle, window = math.radians(5.0), math.radians(175.0), 1
    elif level == 2:
        min_angle, max_angle, window = math.radians(5.0), math.radians(175.0), 3
    elif level == 3:
        min_angle, max_angle, window = math.radians(15.0), math.radians(165.0), 5
    else:
        min_angle = math.radians(min_angle_deg)
        max_angle = math.radians(max_angle_deg)
    window = max(int(window), 1)
    step = max(int(traverse_step), 1)
    angle_increment = (FOV_DEGREES / 180.0) * math.pi / scan_num
    thresh_min: list[float] = []
    thresh_max: list[float] = []
    for i in range(window):
        denom_min = math.sin(math.pi - min_angle - angle_increment * (i + 1))
        denom_max = math.sin(math.pi - max_angle - angle_increment * (i + 1))
        thresh_min.append(math.sin(min_angle) / denom_min if denom_min else 0.0)
        thresh_max.append(math.sin(max_angle) / denom_max if denom_max else 0.0)

    delete: set[int] = set()
    for i in range(0, scan_num, step):
        if distances_mm[i] > MAX_DISTANCE_MM or distances_mm[i] == 0:
            continue
        for search_index in range(i + 1, i + window + 1):
            neighbor = min(max(search_index, 0), scan_num - 1)
            if distances_mm[neighbor] > MAX_DISTANCE_MM or distances_mm[neighbor] == 0:
                continue
            slot = abs(neighbor - i - 1)
            if slot >= len(thresh_min):
                continue
            rate = distances_mm[i] / distances_mm[neighbor]
            if rate < thresh_min[slot] or rate > thresh_max[slot]:
                if distances_mm[i] < distances_mm[neighbor]:
                    delete.add(neighbor)
                else:
                    delete.add(i)
    filtered = list(distances_mm)
    for index in delete:
        filtered[index] = MAX_DISTANCE_MM + 1000
    return filtered


def disturb_filter(
    distances_mm: list[int], *, enabled: bool, threshold_mm: int, point_num: int
) -> list[int]:
    """Drop short runs of points whose neighbor-to-neighbor jump exceeds a threshold."""
    if not enabled or not distances_mm:
        return distances_mm
    point_num = max(int(point_num), 1)
    begin_index = 0
    delete: list[int] = []
    for i in range(len(distances_mm) - 1):
        jump = abs(distances_mm[i] - distances_mm[i + 1])
        if jump > threshold_mm:
            if i - begin_index <= point_num:
                delete.extend(range(begin_index, i + 2))
            begin_index = i
    if not delete:
        return distances_mm
    filtered = list(distances_mm)
    for index in delete:
        if 0 <= index < len(filtered):
            filtered[index] = MAX_DISTANCE_MM + 1000
    return filtered


class ScanParser:
    """Incremental parser for the HE TCP/UDP byte stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._distances: list[int] = []
        self._intensities: list[int] = []
        self._angle_increment: float | None = None
        self._revolution_open = False
        self._start_time_us = 0
        self.area = AreaStatus()

    def feed(self, data: bytes) -> ParseResult:
        self._buffer.extend(data)
        result = ParseResult()
        while True:
            if len(self._buffer) > 1_000_000:
                del self._buffer[:-64]
            header_at = self._find_magic()
            if header_at is None:
                if len(self._buffer) > 4:
                    del self._buffer[:-4]
                break
            if header_at > 0:
                del self._buffer[:header_at]
            if self._buffer.startswith(AREA_MAGIC):
                if len(self._buffer) < AREA_PACKET_SIZE:
                    break
                packet = bytes(self._buffer[:AREA_PACKET_SIZE])
                del self._buffer[:AREA_PACKET_SIZE]
                self.area = AreaStatus(channel=packet[5], output=packet[7])
                result.area = self.area
                continue
            if len(self._buffer) < HEADER_SIZE:
                break
            (
                start_angle,
                end_angle,
                data_size,
                data_position,
                measure_size,
                timestamp,
            ) = struct.unpack_from(">HHHHHH", self._buffer, 4)
            if data_size > measure_size:
                data_size = measure_size
            body_size = HEADER_SIZE + data_size * 4
            if len(self._buffer) < body_size:
                break
            body = bytes(self._buffer[:body_size])
            del self._buffer[:body_size]
            scan = self._consume_range(
                body,
                start_angle=start_angle,
                end_angle=end_angle,
                data_size=data_size,
                data_position=data_position,
                measure_size=measure_size,
                timestamp=timestamp,
            )
            if scan is not None:
                result.scans.append(scan)
        return result

    def _find_magic(self) -> int | None:
        range_at = self._buffer.find(RANGE_MAGIC)
        area_at = self._buffer.find(AREA_MAGIC)
        found = [index for index in (range_at, area_at) if index >= 0]
        if not found:
            return None
        return min(found)

    def _consume_range(
        self,
        body: bytes,
        *,
        start_angle: int,
        end_angle: int,
        data_size: int,
        data_position: int,
        measure_size: int,
        timestamp: int,
    ) -> Scan | None:
        if measure_size == 0 or data_size == 0:
            return None
        if start_angle == SCAN_START_DEG:
            increment = (end_angle - start_angle) / measure_size
            if increment <= 0:
                return None
            self._angle_increment = increment
            count = int(round(FOV_DEGREES / increment))
            begin = self._begin_index(start_angle, data_position, data_size)
            if begin <= 0 or not self._revolution_open or len(self._distances) != count:
                self._distances = [0] * count
                self._intensities = [0] * count
                self._revolution_open = True
                self._start_time_us = timestamp
        if self._angle_increment is None or not self._revolution_open:
            return None
        begin = self._begin_index(start_angle, data_position, data_size)
        needed = begin + data_size
        if needed > len(self._distances):
            self._distances.extend([0] * (needed - len(self._distances)))
            self._intensities.extend([0] * (needed - len(self._intensities)))
        for i in range(data_size):
            distance, intensity = struct.unpack_from("<HH", body, HEADER_SIZE + i * 4)
            if distance > MAX_DISTANCE_MM:
                distance = MAX_DISTANCE_MM + 10000
            if intensity > MAX_INTENSITY:
                intensity = MAX_INTENSITY
            index = begin + i
            if index < 0:
                continue
            self._distances[index] = distance
            self._intensities[index] = intensity
        if (
            end_angle == SCAN_END_DEG
            and data_position == measure_size
            and self._revolution_open
        ):
            if timestamp >= self._start_time_us:
                elapsed = timestamp - self._start_time_us
            else:
                elapsed = 65535 - self._start_time_us + timestamp
            scan = Scan(list(self._distances), list(self._intensities), elapsed)
            self._revolution_open = False
            return scan
        return None

    def _begin_index(self, start_angle: int, data_position: int, data_size: int) -> int:
        assert self._angle_increment is not None
        return int(
            round(
                (start_angle - SCAN_START_DEG) / self._angle_increment
                + data_position
                - data_size
            )
        )
