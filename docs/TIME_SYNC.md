# Time Synchronization

## LiDAR-camera

The camera is physically triggered through Line0. A patched, pinned `livox_ros_driver2` writes the LiDAR sensor timestamp (`pkg.base_time`) to `/dev/shm/agt_livox_timebase` immediately before CustomMsg publication. The camera driver reads that sensor-time record and stamps the image without waiting for a LiDAR ROS callback.

The rejected legacy design matched camera and LiDAR by host-arrival time and could drop images when a pairing window failed. v0.1 does not contain that path.

A stale, repeated or unavailable shared timebase is diagnostic evidence. It must not itself cause a captured camera frame to be dropped.

## GNSS

G70 NAV-PVT has an independent GNSS clock in v0.1. The driver preserves both GNSS sensor UTC and local host receive time. PPS bridging to the LiDAR/camera hardware time domain is deferred to v0.2.
