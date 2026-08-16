# AGT Sensor Capture

ROS 2 Humble acquisition layer for Livox MID360, a Line0-triggered Hikrobot industrial camera, and WHEELTEC G70 RTK GNSS.

The repository has one responsibility: **produce auditable sensor data and synchronization evidence**. FAST-LIVO2, navigation, localization fusion, and accuracy evaluation remain downstream concerns.

## v0.1 synchronization claim

```text
MID360 <---- hardware/driver timebase ----> Hikrobot camera (Line0)

G70 ---- independent GNSS UTC clock (no PPS bridge in v0.1)
```

LiDAR-camera acquisition does **not** use host-arrival nearest-neighbor matching and does not drop a captured camera frame merely because timing evidence is stale/repeated. G70 is read-only at runtime and uses UBX NAV-PVT as its primary data source.

## Canonical topics

The dataset-facing contract is documented in [`docs/TOPIC_CONTRACT.md`](docs/TOPIC_CONTRACT.md). Core topics include:

```text
/agt/sensors/lidar/custom
/agt/sensors/imu/data
/agt/sensors/camera/front/image_raw
/agt/sensors/camera/front/camera_info
/agt/sensors/camera/front/status
/agt/sensors/gnss/fix
/agt/sensors/gnss/velocity
/agt/sensors/gnss/time_reference
/agt/sensors/gnss/status
/agt/sensors/sync/status
```

## Prerequisites

Target platform:

- Ubuntu 22.04
- ROS 2 Humble
- Livox-SDK2 installed for the official `livox_ros_driver2`
- Hikrobot MVS SDK installed on the target host
- MID360 network interface matching `src/agt_capture_bringup/config/mid360_network.json`
- G70 available as `/dev/wheeltec_gnss`
- G70 configured once in u-center for UBX NAV-PVT at 5 Hz

The committed MID360 network JSON intentionally matches the current `agt_navigation_v2` baseline (`host 192.168.1.5`, MID360 `192.168.1.157`). Override `mid360_config:=...` if the host or sensor IP changes.

## Prepare the workspace

The pinned upstream `livox_ros_driver2` does not keep a root `package.xml`; its own build script copies `package_ROS2.xml` before building. Do **not** run that upstream script inside this workspace because it also clears workspace build/install directories.

Use the non-destructive AGT helper instead:

```bash
source /opt/ros/humble/setup.bash

vcs import src < dependencies.repos
./scripts/prepare_livox_ros2.sh src/livox_ros_driver2

colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

The helper is idempotent. It applies the pinned AGT timebase patch and copies the patched `package_ROS2.xml` to `package.xml` so normal `colcon` discovery works.

## Start all sensors

```bash
ros2 launch agt_capture_bringup sensors.launch.py
```

Useful overrides:

```bash
ros2 launch agt_capture_bringup sensors.launch.py \
  mid360_config:=/absolute/path/to/mid360.json \
  camera_params:=/absolute/path/to/camera.yaml \
  g70_port:=/dev/wheeltec_gnss \
  g70_baud:=9600
```

The camera is started two seconds after the other nodes by default so the LiDAR shared timebase normally exists before the first published image.

## Check the live chain

```bash
python3 tools/check_rates.py --duration 60
python3 tools/check_timestamps.py --duration 60

ros2 topic echo /agt/sensors/camera/front/status --once
ros2 topic echo /agt/sensors/gnss/status --once
ros2 topic echo /agt/sensors/sync/status --once
```

Expected nominal rates:

```text
MID360 CustomMsg : 10 Hz
MID360 IMU       : ~200 Hz
Hikrobot camera  : 10 Hz
G70 NAV-PVT      : 5 Hz
```

## Record a canonical rosbag2 session

Run the sensor launch first, then in another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_capture_bringup record.launch.py \
  session_name:=outdoor_001 \
  output_root:=$HOME/datasets/agt_sensor_capture
```

The output has this form:

```text
outdoor_001/
├── session.yaml
└── rosbag2/
    ├── metadata.yaml
    └── *.db3
```

Inspect it with:

```bash
python3 tools/inspect_bag.py $HOME/datasets/agt_sensor_capture/outdoor_001
```

`inspect_bag.py` reads rosbag2 `metadata.yaml` directly, which is compatible with Humble.

## Calibration

This repository does not ship fabricated camera intrinsics or sensor extrinsics. Set `camera_info_url` to the calibrated Hikrobot camera YAML and keep LiDAR-camera extrinsics in the consuming FAST-LIVO2/navigation configuration.

## Acceptance

v0.1 software is only accepted on hardware after both runs are completed:

- [`docs/acceptance/v0.1-static.md`](docs/acceptance/v0.1-static.md): at least 5 minutes static
- [`docs/acceptance/v0.1-motion.md`](docs/acceptance/v0.1-motion.md): at least 5 minutes with straight segments and turns

Until those documents contain real measurements, the repository must not claim that hardware acceptance is complete.

## Design and implementation plan

- [`docs/superpowers/specs/2026-08-16-agt-sensor-capture-design.md`](docs/superpowers/specs/2026-08-16-agt-sensor-capture-design.md)
- [`docs/superpowers/plans/2026-08-16-agt-sensor-capture-v01.md`](docs/superpowers/plans/2026-08-16-agt-sensor-capture-v01.md)

## License

AGT-owned source code is Apache-2.0. External SDKs and upstream dependencies retain their own licenses. GPL FAST-LIVO2/LIV-handhold code is used as architectural reference and is not copied wholesale into this repository.
