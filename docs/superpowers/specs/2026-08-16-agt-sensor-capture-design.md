# AGT Sensor Capture v0.1 Design

Date: 2026-08-16
Status: Approved and implementation-ready
Target repository: `Aldoubt/agt_sensor_capture`
Target platform: Ubuntu 22.04 + ROS 2 Humble

## 1. Purpose

`agt_sensor_capture` is the canonical acquisition layer for AGT robotics datasets. It is intentionally independent from FAST-LIVO2, Nav2, mapping, localization, and benchmarking algorithms.

The v0.1 release establishes a reproducible sensor acquisition chain for:

- Livox MID360 LiDAR + internal IMU
- Hikrobot industrial camera using Line0 external trigger
- WHEELTEC G70 single-antenna RTK GNSS
- rosbag2 recording and acquisition-health evidence

The central rule is:

> Acquisition produces evidence. Algorithms and benchmark repositories produce conclusions.

## 2. v0.1 scope

### 2.1 In scope

1. MID360 acquisition on ROS 2 Humble
2. Hikrobot Line0 external-trigger camera acquisition at stable 10 Hz
3. LiDAR-camera timestamp relationship following the FAST-LIVO2/LIV-handhold driver-layer model rather than host-arrival nearest-neighbor matching
4. G70 acquisition at 5 Hz using UBX NAV-PVT as the primary protocol
5. Canonical `/agt/sensors/*` topics
6. Sensor rate, timestamp monotonicity, frame-gap, checksum and synchronization diagnostics
7. One-command rosbag2 recording with session metadata
8. Dataset contract suitable for `agt_navigation_v2` and `lio_benchmark_tools`

### 2.2 Explicitly out of scope

- FAST-LIVO2 algorithm source
- Nav2 or vehicle control
- GNSS/LIO fusion
- ATE/RPE evaluation
- G70 PPS integration with the LiDAR/camera hardware clock
- PPK/raw-observation processing
- online nearest-neighbor timestamp correction
- automatic NTRIP client redesign

G70 PPS integration is deferred to v0.2 after v0.1 acquisition stability is proven.

## 3. Repository boundaries

```text
agt_sensor_capture/
├── README.md
├── LICENSE
├── dependencies.repos
├── src/
│   ├── agt_capture_msgs/
│   ├── agt_capture_bringup/
│   ├── agt_hik_camera_driver/
│   ├── agt_g70_driver/
│   └── agt_capture_monitor/
├── config/
├── tools/
├── docs/
│   ├── TOPIC_CONTRACT.md
│   ├── TIME_SYNC.md
│   ├── G70_PROTOCOL.md
│   ├── HARDWARE.md
│   └── superpowers/specs/
└── tests/
```

The repository does not vendor FAST-LIVO2 or navigation stacks.

`livox_ros_driver2` remains an external dependency where possible. Small AGT-owned integration code may expose the LiDAR timebase required by the camera, but the navigation algorithm is not part of this repository.

## 4. Licensing rule

Repository license target: Apache-2.0.

The implementation must not wholesale copy GPL-licensed `LIV_handhold` / `LIV_handhold_2` camera-driver source into an otherwise permissively licensed AGT repository.

The FAST-LIVO2/LIV-handhold repositories are treated as behavioral references for the synchronization architecture. AGT camera and G70 acquisition code will be implemented independently against the public SDK/protocol interfaces.

This keeps repository ownership and licensing boundaries auditable.

## 5. Canonical topic contract

```text
/agt/sensors/lidar/custom
    livox_ros_driver2/msg/CustomMsg

/agt/sensors/imu/data
    sensor_msgs/msg/Imu

/agt/sensors/camera/front/image_raw
    sensor_msgs/msg/Image

/agt/sensors/camera/front/camera_info
    sensor_msgs/msg/CameraInfo

/agt/sensors/gnss/fix
    sensor_msgs/msg/NavSatFix

/agt/sensors/gnss/velocity
    geometry_msgs/msg/TwistWithCovarianceStamped

/agt/sensors/gnss/time_reference
    sensor_msgs/msg/TimeReference

/agt/sensors/gnss/status
    agt_capture_msgs/msg/GnssStatus

/agt/sensors/sync/status
    diagnostic_msgs/msg/DiagnosticArray

/diagnostics
    diagnostic_msgs/msg/DiagnosticArray
```

Vendor topic names may exist internally, but system-level consumers use the canonical AGT names.

## 6. LiDAR-camera synchronization model

### 6.1 Rejected design

The legacy `fastlivo2_platform` implementation associates image and LiDAR data by host-arrival time and may drop an image when no LiDAR ROS message falls inside a pairing window.

This design is not carried into v0.1.

### 6.2 v0.1 design

The Line0 hardware trigger remains the physical camera trigger. The ROS camera driver reads a LiDAR-derived hardware timebase exposed by the acquisition layer and uses it as the image timestamp.

```text
hardware synchronizer
        │
   ┌────┴────┐
   │         │
MID360    Hikrobot
   │        Line0
   │         │
LiDAR       camera
sensor      frame
stamp        │
   │         │
   └── shared timebase ──> image.header.stamp
```

There is no ROS host-arrival nearest-neighbor synchronizer in this path.

### 6.3 Shared timebase transport

Default path:

```text
/dev/shm/agt_livox_timebase
```

Recommended shared structure:

```cpp
struct SharedTimebaseV1
{
  uint32_t magic;
  uint32_t version;
  uint64_t sequence;
  uint64_t lidar_stamp_ns;
  uint64_t host_update_steady_ns;
};
```

The sequence and host update time are diagnostic evidence. They allow the camera driver to detect stale or repeated LiDAR timebase values without waiting for or dropping frames.

A synchronization fault is reported; it is not converted into an acquisition drop.

## 7. Hikrobot camera driver

`agt_hik_camera_driver` is a minimal ROS 2 Humble driver implemented around the Hikrobot MVS SDK.

Required v0.1 configuration:

```yaml
trigger_mode: true
trigger_source: Line0
trigger_activation: FallingEdge
exposure_auto: Off
exposure_time_us: 5000.0
gain_auto: Continuous
pixel_format: RGB8Packed
expected_rate_hz: 10.0
frame_id: camera_front_optical_frame
```

The capture path must not wait on LiDAR ROS callbacks or a condition variable.

At minimum, diagnostics track:

```text
SDK frame number
trigger index when available
SDK receive count
SDK timeout count
conversion failure count
publish count
frame-number gap count
timestamp rollback count
repeated/stale timebase count
```

The acceptance metric is the SDK/published image rate, not merely whether images appear in RViz.

## 8. G70 protocol decision

### 8.1 Available protocols

The G70 supports NMEA-0183, RTCM 3.3 and UBX. The two supplied ROS examples represent two acquisition approaches:

- NMEA text parsing using a modified `nmea_navsat_driver`
- UBX binary parsing using the ROS 2 `ublox_gps` stack

### 8.2 Decision

**v0.1 uses UBX NAV-PVT as the primary G70 data protocol.**

NMEA is not the primary benchmark path. It may be added later as a compatibility/fallback mode.

### 8.3 Why UBX is selected

For localization ground-truth acquisition, NAV-PVT carries in one coherent binary measurement:

- UTC date and time
- nanosecond sub-second field
- GPS time-of-week
- fix type and fix-valid flags
- carrier-phase solution state (none / float / fixed)
- number of satellites
- latitude / longitude
- ellipsoid and MSL height
- horizontal and vertical accuracy estimates
- N/E/D velocity
- speed accuracy
- motion heading and heading accuracy

This is materially better suited to later trajectory association and truth-quality filtering than a GGA/VTG-only data path.

### 8.4 Why the supplied NMEA driver is not used as the canonical path

The supplied NMEA code is useful as a functional example, but it is not ideal as benchmark evidence because it:

- timestamps standard ROS messages primarily at host processing time
- reconstructs UTC date from the host's current UTC date when parsing time-of-day
- converts NMEA time with integer-second handling in the current supplied parser
- uses configured/default error values when receiver error estimates are unavailable
- maps RTK state into standard `NavSatStatus`, while the custom GGA message is needed to retain the explicit GGA quality code

Those behaviors are acceptable for visualization and basic navigation, but unnecessarily weaken the truth timestamp/quality contract.

### 8.5 Why the supplied full `ublox_gps` node is not used unchanged

The supplied ROS 2 u-blox stack is substantially richer than v0.1 needs. It also contains active receiver-configuration behavior: rate configuration, dynamic/fix model configuration, signal configuration and message-rate configuration.

The supplied WHEELTEC UBX YAML also contains survey-in/TMODE-oriented settings inherited from high-precision u-blox reference-station examples. That is not the desired default behavior for a mobile G70 rover acquisition node.

For a benchmark capture layer, the GNSS reader should be read-only by default and must not unexpectedly change the receiver operating mode during startup.

## 9. G70 driver design

`agt_g70_driver` is therefore a small AGT-owned, read-only UBX NAV-PVT parser.

### 9.1 Receiver setup

Receiver configuration is intentionally separated from runtime acquisition.

One-time configuration in u-center:

```text
protocol output: UBX
required message: UBX-NAV-PVT
navigation output rate: 5 Hz
serial baud: initially 9600 unless field testing requires a higher value
```

RAWX and SFRBX are not required for v0.1 RTK trajectory truth. They are deferred to a future raw-observation/PPK feature.

### 9.2 Parser

The runtime parser:

1. opens `/dev/wheeltec_gnss`
2. scans for UBX sync bytes `0xB5 0x62`
3. reads class, message ID and payload length
4. validates the UBX Fletcher checksum
5. accepts NAV-PVT (`class=0x01`, `id=0x07`)
6. validates payload length/version assumptions
7. decodes fields without sending configuration commands to the receiver
8. records host receive time as separate evidence
9. publishes canonical ROS messages

Malformed packets increment diagnostics and are discarded without restarting the complete acquisition stack.

### 9.3 Timestamp semantics

Three concepts remain distinct:

```text
sensor_time       GNSS UTC reconstructed from NAV-PVT
host_receive_time local host time when the complete NAV-PVT frame is received
bag_record_time   rosbag2 recorder receipt time
```

When NAV-PVT date/time is valid and fully resolved:

```text
NavSatFix.header.stamp = sensor_time
velocity.header.stamp  = sensor_time
GnssStatus.header.stamp = sensor_time
TimeReference.time_ref = sensor_time
TimeReference.header.stamp = host_receive_time
```

If NAV-PVT time validity is insufficient, the driver must not silently claim hardware-quality GNSS time. The status message marks time invalid; downstream benchmark tooling can reject those samples.

### 9.4 GNSS status message

Proposed `agt_capture_msgs/msg/GnssStatus.msg`:

```text
std_msgs/Header header
builtin_interfaces/Time host_receive_time

uint32 i_tow_ms
uint32 time_accuracy_ns
bool time_valid
bool fully_resolved

uint8 fix_type
bool gnss_fix_ok
bool differential_solution

uint8 CARRIER_NONE=0
uint8 CARRIER_FLOAT=1
uint8 CARRIER_FIXED=2
uint8 carrier_solution

uint8 num_satellites
float64 horizontal_accuracy_m
float64 vertical_accuracy_m
float64 speed_accuracy_mps
float64 motion_heading_rad
float64 heading_accuracy_rad
```

The benchmark repository can select `carrier_solution == CARRIER_FIXED` instead of inferring RTK quality from a generic NavSat status code.

## 10. G70 rate

v0.1 target rate is 5 Hz.

A single NAV-PVT packet at 5 Hz is small enough for the default serial link and avoids the bandwidth pressure caused by simultaneously streaming many NMEA sentences or raw-observation messages.

The capture monitor reports the observed rate and packet/checksum loss rather than assuming the configured rate was achieved.

## 11. Capture monitor

`agt_capture_monitor` observes only. It never modifies sensor timestamps or drops data.

It reports:

```text
LiDAR rate and timestamp monotonicity
IMU rate and timestamp monotonicity
camera SDK/publish rates and frame gaps
camera shared-timebase freshness
GNSS NAV-PVT rate
GNSS checksum failures
GNSS UTC validity
GNSS RTK fixed/float/no-carrier distribution
```

## 12. rosbag2 session contract

A capture session records at least:

```text
/agt/sensors/lidar/custom
/agt/sensors/imu/data
/agt/sensors/camera/front/image_raw
/agt/sensors/camera/front/camera_info
/agt/sensors/gnss/fix
/agt/sensors/gnss/velocity
/agt/sensors/gnss/time_reference
/agt/sensors/gnss/status
/agt/sensors/sync/status
/diagnostics
/tf
/tf_static
```

Session metadata records the synchronization claim explicitly:

```yaml
sync:
  lidar_camera: hardware_trigger_driver_timebase
  gnss: independent_gnss_clock_no_pps_bridge
```

No v0.1 dataset may claim three-sensor hardware synchronization.

## 13. Acceptance criteria

### 13.1 Static test

Duration: at least 5 minutes.

Expected:

```text
camera published rate: 9.9-10.1 Hz average
camera sustained frame gaps: 0
camera timestamp rollback: 0
LiDAR rate: approximately 10 Hz
IMU rate: approximately 200 Hz
LiDAR/IMU timestamp rollback: 0
GNSS rate: approximately 5 Hz
GNSS UBX checksum failures: 0 during a healthy wired run
GNSS timestamp rollback: 0
```

### 13.2 Motion test

Duration: at least 5 minutes with straight segments and turns.

The bag must retain valid camera frames, LiDAR/IMU data, GNSS UTC, RTK carrier state and accuracy estimates through motion.

### 13.3 Dataset acceptance

`ros2 bag info` message counts must be consistent with the configured rates and duration. A generated acquisition report accompanies the bag.

## 14. Planned implementation sequence

1. initialize repository and CI/build skeleton
2. add `agt_capture_msgs`
3. freeze topic contract and metadata schema
4. integrate MID360 and shared LiDAR timebase writer
5. implement clean ROS 2 Hikrobot Line0 camera driver
6. validate stable 10 Hz camera publication
7. implement read-only G70 UBX NAV-PVT parser
8. add canonical GNSS publishers and `GnssStatus`
9. add capture monitor
10. add rosbag2 session launcher and metadata
11. add rate/timestamp inspection tools
12. execute 5-minute static acceptance
13. execute 5-minute motion acceptance
14. document dataset handoff to `lio_benchmark_tools`

## 15. Future v0.2

After v0.1 is stable:

- connect G70 PPS into the timing architecture
- define common hardware clock/time-reference semantics
- measure LiDAR-camera-GNSS residual offset
- add optional RAWX/SFRBX logging for PPK research if needed
- add a NMEA fallback input mode only if field compatibility requires it
