# AGT Sensor Capture

ROS 2 Humble acquisition layer for synchronized Livox MID360 + Hikrobot camera data and read-only WHEELTEC G70 RTK GNSS evidence.

## v0.1 scope

- MID360 CustomMsg + internal IMU
- Hikrobot Line0 external trigger, target 10 Hz
- LiDAR-derived shared sensor timebase for camera timestamps
- G70 UBX NAV-PVT at 5 Hz
- canonical `/agt/sensors/*` topics
- rosbag2 recording and acquisition diagnostics

FAST-LIVO2, Nav2, localization fusion and accuracy evaluation stay outside this repository.

## Canonical interfaces

See [`docs/TOPIC_CONTRACT.md`](docs/TOPIC_CONTRACT.md) and [`docs/TIME_SYNC.md`](docs/TIME_SYNC.md).

## External dependency

`dependencies.repos` pins the official `Livox-SDK/livox_ros_driver2` revision used by v0.1.

## Build

```bash
source /opt/ros/humble/setup.bash
vcs import src < dependencies.repos
colcon build --symlink-install
```

The Hikrobot camera package additionally requires the Hikrobot MVS SDK installed on the target host.
