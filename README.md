# Hinson HE lidar

Viam camera module for Hinson HE (XingSong) lidars. It speaks the same TCP/UDP
protocol as the Hinson ROS driver: connect, optionally write scan parameters,
send the `RAuto` start command, and assemble `HISN` range packets into one
320 degree revolution (20 degrees through 340 degrees on the sensor cover).

The module is Python. The vendor driver is C++, but it is built as a ROS node
on Boost.Asio. The wire format is small enough to implement directly, and the
Viam Python SDK is the straightforward way to ship a camera model.

`GetPointCloud` returns a binary PCD in millimeters, in the same units as the
Viam RPLidar camera. `GetImages` returns a top-down PNG of that scan so the
camera card in the Viam app has a picture.

## Configure

Add a local module whose executable is `run.sh`, then add a camera:

```json
{
  "modules": [
    {
      "name": "hinson-he",
      "executable_path": "/path/to/hinson_he_lidar/run.sh"
    }
  ],
  "components": [
    {
      "name": "he-lidar",
      "api": "rdk:component:camera",
      "model": "viam-labs:camera:hinson-he",
      "attributes": {
        "laser_ip": "192.168.1.88",
        "laser_port": 8080
      }
    }
  ]
}
```

`run.sh` installs `requirements.txt` into `deps/` on first launch.

| Attribute | Default | Description |
| --- | --- | --- |
| `laser_ip` | `192.168.1.88` | Lidar address |
| `laser_port` | `8080` | Lidar port |
| `use_udp` | `false` | Use UDP instead of TCP |
| `change_param` | `false` | Write scan settings on connect. When false, the lidar keeps the parameters from the Windows configuration tool. |
| `spin_frequency_hz` | `10` | `10` or `20`. Used only when `change_param` is true. |
| `angle_increment` | `0.050` | `0.025`, `0.050`, `0.100`, `0.250`, or `0.500` degrees. `0.025` forces 10 Hz, matching the vendor command. |
| `noise_filter_level` | `1` | `0` to `3` |
| `forward_axis` | `x` | `x` matches the ROS driver with `y_direction` false. `y` matches `y_direction` true. |
| `angle_offset_deg` | `0` | Extra rotation applied after `forward_axis` |
| `start_angle_deg`, `end_angle_deg` | `0`, `360` | Keep returns whose sensor-cover angle is inside this window |
| `min_range_mm`, `max_range_mm` | `50`, `50000` | Returns outside this span are omitted |
| `shadows_filter_level` | `0` | Trailing-edge filter. `0` off, `-1` uses the angle and window attributes, `1`–`3` are the vendor presets. |
| `disturb_filter_enable` | `false` | Crosstalk filter |
| `block_enable` | `false` | Periodically request obstacle-area packets |
| `area_channel` | `0` | Area channel requested when `block_enable` is true |

`do_command` with `{"channel": 2}` changes the area channel while the resource
is running. `get_status` reports connection state, scan age, point count, and
the three area bits.

Points are in the lidar frame. With `forward_axis` `x`, sensor-cover angle 20
degrees (the first return) is at -160 degrees in the standard mathematical
frame, which is the ROS driver's x-forward scan.

The protocol and filter behavior follow the Hinson ROS driver, BSD-3-Clause,
Copyright (c) 2022, HINSON. This module is Apache-2.0.
