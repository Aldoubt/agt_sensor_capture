# Time Synchronization

## LiDAR-camera

The camera is physically triggered through Line0. A patched, pinned `livox_ros_driver2` writes the LiDAR sensor timestamp (`pkg.base_time`) to `/dev/shm/agt_livox_timebase` immediately before CustomMsg publication. The camera driver reads that sensor-time record and stamps the image without waiting for a LiDAR ROS callback.

The rejected legacy design matched camera and LiDAR by host-arrival time and could drop images when a pairing window failed. v0.1 does not contain that path.

A stale, repeated or unavailable shared timebase is diagnostic evidence. It must not itself cause a captured camera frame to be dropped.

A stale or repeated but structurally valid timebase is still attached to the captured frame and counted in `CameraStatus`; an unavailable/invalid timebase falls back to host system time and sets `timebase_valid=false`. This preserves the raw frame while making the degraded timing claim explicit. The camera reopens the shared timebase lazily, so starting it before the Livox writer does not require restarting the camera node.

## Preparing the pinned Livox ROS 2 dependency

Import the exact upstream revision declared in `dependencies.repos`, then run the non-destructive AGT helper:

```bash
vcs import src < dependencies.repos
scripts/prepare_livox_ros2.sh src/livox_ros_driver2
```

`prepare_livox_ros2.sh` is idempotent. It applies the AGT patch and copies the patched `package_ROS2.xml` to `package.xml`, avoiding the upstream `build.sh` behavior that clears workspace build/install directories. The patch does the minimal ROS 2 integration changes: declares the `agt_timebase` dependency, defaults this pin to C++17/Humble, constructs one shared-timebase writer, and writes the exact `StoragePacket::base_time` before publishing each Livox CustomMsg. It does not subscribe to another ROS topic and does not rewrite the Livox message timestamp.

The default shared record is `/dev/shm/agt_livox_timebase` and contains a magic/version, a monotonically increasing sequence, the LiDAR sensor timestamp in nanoseconds, and a host steady-clock update timestamp used only for freshness diagnostics.

## GNSS

G70 NAV-PVT has an independent GNSS clock in v0.1. The driver preserves both GNSS sensor UTC and local host receive time. PPS bridging to the LiDAR/camera hardware time domain is deferred to v0.2.
