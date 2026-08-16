# Hardware Assumptions

## MID360 and camera

- Livox MID360
- Hikrobot industrial camera supported by the installed MVS SDK
- camera external trigger source: `Line0`
- trigger activation: `FallingEdge`
- target camera rate: 10 Hz
- fixed exposure baseline: 5000 us
- initial pixel format: `RGB8Packed`

The v0.1 camera driver intentionally has no LiDAR ROS subscription. The physical trigger establishes the exposure event relationship; the pinned Livox integration exposes `StoragePacket::base_time` through `/dev/shm/agt_livox_timebase`, and the camera snapshots that driver-layer timebase after each received SDK frame.

Because v0.1 uses `RGB8Packed`, the capture path does not resize or run OpenCV conversion. It copies the SDK frame into the ROS image, releases the SDK buffer, and publishes. This keeps the 10 Hz acquisition path small and removes the old synchronization wait/drop path.

Install the Hikrobot MVS SDK on the target before building `agt_hik_camera_driver`. Typical Linux locations used by CMake are `/opt/MVS/include` and `/opt/MVS/lib/64` (x86_64) or `/opt/MVS/lib/aarch64` (ARM64).

`camera_info_url` is empty by default: v0.1 does not ship fabricated intrinsics. Set it to the calibrated camera-info YAML before using consumers that require the ROS CameraInfo calibration.

## G70

- WHEELTEC G70 single-antenna RTK receiver
- Linux device alias: `/dev/wheeltec_gnss`
- UBX NAV-PVT at 5 Hz
- default serial baud: 9600
- G70 PPS is not connected into the common timing architecture in v0.1

The supplied WHEELTEC examples include udev rules for several supported USB-serial identities. The AGT runtime driver relies on the stable `/dev/wheeltec_gnss` alias but does not install privileged udev changes automatically.
