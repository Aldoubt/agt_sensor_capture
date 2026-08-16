# Time Synchronization

## LiDAR-camera

The camera is physically triggered through Line0. A patched, pinned `livox_ros_driver2` writes the LiDAR sensor timestamp (`pkg.base_time`) to `/dev/shm/agt_livox_timebase` immediately before CustomMsg publication. The camera driver reads that sensor-time record and stamps the image without waiting for a LiDAR ROS callback.

The rejected legacy design matched camera and LiDAR by host-arrival time and could drop images when a pairing window failed. v0.1 does not contain that path.

A stale, repeated or unavailable shared timebase is diagnostic evidence. It must not itself cause a captured camera frame to be dropped.

## Applying the pinned Livox integration patch

Import the exact upstream revision declared in `dependencies.repos`, then apply the AGT patch:

```bash
vcs import src < dependencies.repos
scripts/apply_livox_patch.sh src/livox_ros_driver2
```

The helper is idempotent: a second invocation reports that the patch is already applied. The patch does only four integration changes for ROS 2: declares the `agt_timebase` dependency, compiles the ROS 2 driver as C++17, constructs one shared-timebase writer, and writes the exact `StoragePacket::base_time` before publishing each Livox CustomMsg. It does not subscribe to another ROS topic and does not rewrite the Livox message timestamp.

The default shared record is `/dev/shm/agt_livox_timebase` and contains a magic/version, a monotonically increasing sequence, the LiDAR sensor timestamp in nanoseconds, and a host steady-clock update timestamp used only for freshness diagnostics.

## GNSS

G70 NAV-PVT has an independent GNSS clock in v0.1. The driver preserves both GNSS sensor UTC and local host receive time. PPS bridging to the LiDAR/camera hardware time domain is deferred to v0.2.
