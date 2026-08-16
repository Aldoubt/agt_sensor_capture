# Topic Contract

This repository publishes canonical AGT sensor interfaces. Vendor-native topic names may exist internally, but bags and downstream AGT software use the names below.

| Topic | Type | Meaning |
|---|---|---|
| `/agt/sensors/lidar/custom` | `livox_ros_driver2/msg/CustomMsg` | MID360 raw CustomMsg |
| `/agt/sensors/imu/data` | `sensor_msgs/msg/Imu` | MID360 internal IMU |
| `/agt/sensors/camera/front/image_raw` | `sensor_msgs/msg/Image` | Front Hikrobot image |
| `/agt/sensors/camera/front/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration |
| `/agt/sensors/camera/front/status` | `agt_capture_msgs/msg/CameraStatus` | Camera acquisition evidence |
| `/agt/sensors/gnss/fix` | `sensor_msgs/msg/NavSatFix` | G70 position |
| `/agt/sensors/gnss/velocity` | `geometry_msgs/msg/TwistWithCovarianceStamped` | G70 N/E/D velocity mapped to ENU ROS axes |
| `/agt/sensors/gnss/time_reference` | `sensor_msgs/msg/TimeReference` | GNSS UTC and host-receive relationship |
| `/agt/sensors/gnss/status` | `agt_capture_msgs/msg/GnssStatus` | UBX time/fix/RTK quality evidence |
| `/agt/sensors/sync/status` | `diagnostic_msgs/msg/DiagnosticArray` | Cross-sensor acquisition/sync evidence |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Standard ROS diagnostics |

v0.1 explicitly does not claim that GNSS is hardware synchronized to MID360/camera.
