import math
import struct

import pytest

from hinson_he.camera import parse_config
from hinson_he.cloud import encode_pcd, project_points, sensor_angle_deg
from hinson_he.protocol import (
    MAX_DISTANCE_MM,
    AreaStatus,
    Scan,
    ScanParser,
    build_area_command,
    build_param_command,
    crc16,
    disturb_filter,
    shadows_filter,
)


def range_packet(start, end, measure_size, data_position, distances, intensities, timestamp=0):
    data_size = len(distances)
    header = bytearray(16)
    header[0:4] = b"HISN"
    struct.pack_into(">HHHHHH", header, 4, start, end, data_size, data_position, measure_size, timestamp)
    body = bytearray()
    for distance, intensity in zip(distances, intensities):
        body += struct.pack("<HH", distance, intensity)
    return bytes(header) + bytes(body)


def eight_point_scan():
    """Two sectors at 40 degrees per point, eight points across 320 degrees."""
    first = range_packet(20, 180, 4, 4, [1000, 2000, 0, 60000], [10, 20, 30, 40], timestamp=100)
    second = range_packet(180, 340, 4, 4, [1500, 49, 50000, 3000], [11, 12, 13, 14], timestamp=900)
    parser = ScanParser()
    assert parser.feed(b"\x00\x01" + first).scans == []
    scans = parser.feed(second).scans
    assert len(scans) == 1
    assert scans[0].distances_mm == [1000, 2000, 0, 60000, 1500, 49, 50000, 3000]
    assert scans[0].intensities[1] == 20
    assert scans[0].time_increment_us == 800
    return scans[0]


def test_crc_and_param_command_layout():
    command = build_param_command(spin_frequency_hz=20, angle_increment="0.050", noise_filter_level=1)
    assert command[:5] == b"SCtrl"
    assert command[7] == 0x01
    assert command[8] == 0x01
    assert command[9] == 0x01
    assert command[10] == crc16(command[:10]) & 0xFF
    assert command[11] == (crc16(command[:10]) >> 8) & 0xFF

    fine = build_param_command(spin_frequency_hz=20, angle_increment="0.025", noise_filter_level=2)
    assert fine[7] == 0x00
    assert fine[8] == 0x00
    assert fine[9] == 0x02


def test_parser_assembles_a_revolution_and_area_packet():
    scan = eight_point_scan()
    assert len(scan.distances_mm) == 8

    area = bytearray(13)
    area[0:5] = b"WSimu"
    area[5] = 4
    area[7] = 0x05
    parser = ScanParser()
    parsed = parser.feed(bytes(area))
    assert parsed.area == AreaStatus(channel=4, output=0x05)
    assert parsed.area.area1 is True
    assert parsed.area.area2 is False
    assert parsed.area.area3 is True

    area_command = build_area_command(channel=4, channel_group=2)
    assert area_command[:5] == b"WSimu"
    assert area_command[6] == 4
    assert area_command[16] == crc16(area_command[:16]) & 0xFF


def test_split_packets_and_timestamp_wrap():
    packet = range_packet(20, 340, 1, 1, [1000], [1], timestamp=10)
    # A single packet whose sector is the whole 320 degree window: increment is 320.
    # scan count = 1. begin = 0.
    parser = ScanParser()
    assert parser.feed(packet[:10]).scans == []
    scans = parser.feed(packet[10:]).scans
    assert len(scans) == 1
    assert scans[0].distances_mm == [1000]

    first = range_packet(20, 180, 4, 4, [1, 1, 1, 1], [0, 0, 0, 0], timestamp=65000)
    second = range_packet(180, 340, 4, 4, [2, 2, 2, 2], [0, 0, 0, 0], timestamp=100)
    parser = ScanParser()
    parser.feed(first)
    scan = parser.feed(second).scans[0]
    assert scan.time_increment_us == 65535 - 65000 + 100


def test_projection_matches_ros_x_forward():
    scan = eight_point_scan()
    points = project_points(
        scan,
        forward_axis="x",
        angle_offset_deg=0,
        start_angle_deg=0,
        end_angle_deg=360,
        min_range_mm=50,
        max_range_mm=50000,
    )
    # index 0 is sensor 20 deg -> -160 deg. index 2 is 0 mm, index 3 is over range.
    assert sensor_angle_deg(0, 8) == pytest.approx(20.0)
    assert len(points) == 5
    x, y, z, intensity = points[0]
    assert z == 0
    assert intensity == 10
    assert x == pytest.approx(1000 * math.cos(math.radians(-160)))
    assert y == pytest.approx(1000 * math.sin(math.radians(-160)))

    y_points = project_points(
        Scan([1000], [7]),
        forward_axis="y",
        angle_offset_deg=0,
        start_angle_deg=0,
        end_angle_deg=360,
        min_range_mm=0,
        max_range_mm=50000,
    )
    # one point, increment 320, sensor angle 20 -> y-forward offset -90 => -70 deg
    assert y_points[0][0] == pytest.approx(1000 * math.cos(math.radians(-70)))
    assert y_points[0][1] == pytest.approx(1000 * math.sin(math.radians(-70)))


def test_pcd_binary_round_trip():
    raw = encode_pcd([(10.0, -20.0, 0.0, 9), (1.5, 2.5, 3.5, 0)])
    header, payload = raw.split(b"DATA binary\n", 1)
    assert b"POINTS 2" in header
    assert struct.unpack("<ffffff", payload) == pytest.approx((10.0, -20.0, 0.0, 1.5, 2.5, 3.5))


def test_filters_and_config():
    untouched = shadows_filter([100, 200, 300], level=0, min_angle_deg=5, max_angle_deg=175, window=2, traverse_step=1)
    assert untouched == [100, 200, 300]

    spiked = [1000] * 20
    spiked[10] = 100
    filtered = shadows_filter(spiked, level=1, min_angle_deg=5, max_angle_deg=175, window=1, traverse_step=1)
    assert MAX_DISTANCE_MM + 1000 in filtered

    disturbed = disturb_filter([1000, 1000, 2000, 1000, 1000], enabled=True, threshold_mm=80, point_num=8)
    assert disturbed[2] == MAX_DISTANCE_MM + 1000

    lidar, view = parse_config({"laser_ip": "10.0.0.8", "change_param": True, "spin_frequency_hz": 20})
    assert lidar.address == "10.0.0.8"
    assert lidar.spin_frequency_hz == 20
    assert view["forward_axis"] == "x"
    with pytest.raises(ValueError):
        parse_config({"change_param": True, "angle_increment": "0.010"})
    with pytest.raises(ValueError):
        parse_config({"forward_axis": "z"})
