# AGT Sensor Capture v0.1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS 2 Humble acquisition repository that records MID360 LiDAR/IMU, a Line0-triggered Hikrobot camera with a LiDAR-derived driver timebase, and read-only WHEELTEC G70 UBX NAV-PVT data on canonical `/agt/sensors/*` topics.

**Architecture:** Keep acquisition independent from FAST-LIVO2 and navigation. Pin official `livox_ros_driver2`, apply a small auditable AGT patch that writes `pkg.base_time` into a versioned shared-memory record, let the camera read that timebase without waiting on ROS callbacks, and implement G70 as a read-only NAV-PVT parser that never configures the receiver at runtime.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, C++17, ament_cmake, rclcpp, sensor_msgs, geometry_msgs, diagnostic_msgs, cv_bridge/OpenCV, Hikrobot MVS SDK, POSIX mmap/termios, Python 3, pytest, ament_cmake_gtest.

## Global Constraints

- Repository: `Aldoubt/agt_sensor_capture`, Apache-2.0.
- Do not wholesale copy GPL `LIV_handhold` / `LIV_handhold_2` code.
- Do not vendor FAST-LIVO2/Nav2/mapping/localization/benchmark algorithms.
- LiDAR-camera synchronization must not use host-arrival nearest-neighbor matching.
- Synchronization faults are diagnostic evidence, not camera-drop conditions.
- G70 is read-only at runtime; primary protocol UBX NAV-PVT at 5 Hz.
- G70 PPS/common hardware-clock integration is deferred to v0.2.
- Canonical outputs use `/agt/sensors/*`; v0.1 must not claim three-sensor hardware synchronization.
- Pin `Livox-SDK/livox_ros_driver2` at `4a1def929e5b59c7a8122d19fce6efba581ce9f7`.

---

### Task 1: Repository skeleton, messages, and topic contract

**Files:**
- Create: `README.md`, `LICENSE`, `.gitignore`, `dependencies.repos`
- Create: `docs/TOPIC_CONTRACT.md`, `docs/TIME_SYNC.md`, `docs/G70_PROTOCOL.md`, `docs/HARDWARE.md`
- Create: `src/agt_capture_msgs/{CMakeLists.txt,package.xml}`
- Create: `src/agt_capture_msgs/msg/GnssStatus.msg`
- Create: `src/agt_capture_msgs/msg/CameraStatus.msg`
- Create: `test/test_topic_contract.py`

**Interfaces:** Produces custom messages and the frozen canonical topic names.

- [ ] **Step 1: Write the failing topic-contract test**

```python
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    "/agt/sensors/lidar/custom", "/agt/sensors/imu/data",
    "/agt/sensors/camera/front/image_raw",
    "/agt/sensors/camera/front/camera_info",
    "/agt/sensors/gnss/fix", "/agt/sensors/gnss/velocity",
    "/agt/sensors/gnss/time_reference", "/agt/sensors/gnss/status",
    "/agt/sensors/sync/status",
}
def test_topic_contract_contains_all_canonical_topics():
    text = (ROOT / "docs" / "TOPIC_CONTRACT.md").read_text()
    assert not sorted(t for t in EXPECTED if t not in text)
```

- [ ] **Step 2: Verify RED**

```bash
python3 -m pytest -q test/test_topic_contract.py
```

Expected: fail because `TOPIC_CONTRACT.md` does not exist.

- [ ] **Step 3: Add messages**

`GnssStatus.msg`:

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

`CameraStatus.msg`:

```text
std_msgs/Header header
uint64 sdk_receive_count
uint64 sdk_timeout_count
uint64 conversion_failure_count
uint64 publish_count
uint64 frame_gap_count
uint64 timestamp_rollback_count
uint64 stale_timebase_count
uint64 repeated_timebase_count
uint64 last_frame_number
uint64 last_trigger_index
float64 sdk_rate_hz
float64 publish_rate_hz
bool timebase_valid
```

- [ ] **Step 4: Pin Livox dependency**

```yaml
repositories:
  livox_ros_driver2:
    type: git
    url: https://github.com/Livox-SDK/livox_ros_driver2.git
    version: 4a1def929e5b59c7a8122d19fce6efba581ce9f7
```

- [ ] **Step 5: Build/test GREEN**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_capture_msgs --event-handlers console_direct+
python3 -m pytest -q test/test_topic_contract.py
```

Expected: build succeeds and test passes.

- [ ] **Step 6: Commit**

```bash
git add README.md LICENSE .gitignore dependencies.repos docs src/agt_capture_msgs test
git commit -m "feat: initialize sensor capture interfaces"
```

---

### Task 2: Versioned shared LiDAR timebase

**Files:**
- Create: `src/agt_timebase/{CMakeLists.txt,package.xml}`
- Create: `src/agt_timebase/include/agt_timebase/shared_timebase.hpp`
- Create: `src/agt_timebase/src/shared_timebase.cpp`
- Create: `src/agt_timebase/test/test_shared_timebase.cpp`

**Interfaces:**

```cpp
namespace agt_timebase {
inline constexpr uint32_t kMagic = 0x41475431U;
inline constexpr uint32_t kVersion = 1U;
inline constexpr const char * kDefaultPath = "/dev/shm/agt_livox_timebase";
struct SharedTimebaseV1 {
  uint32_t magic{kMagic};
  uint32_t version{kVersion};
  uint64_t sequence{0};
  uint64_t lidar_stamp_ns{0};
  uint64_t host_update_steady_ns{0};
};
}
```

- [ ] **Step 1: Write failing round-trip, sequence, invalid-magic/version tests**

```cpp
TEST(SharedTimebase, RoundTripPreservesStampAndSequence) {
  const auto path = std::filesystem::temp_directory_path() / "agt_timebase_test.bin";
  std::filesystem::remove(path);
  agt_timebase::SharedTimebaseWriter writer(path.string());
  agt_timebase::SharedTimebaseReader reader(path.string());
  writer.write(123456789ULL, 987654321ULL);
  auto value = reader.read();
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(value->lidar_stamp_ns, 123456789ULL);
  EXPECT_EQ(value->sequence, 1ULL);
  EXPECT_EQ(value->host_update_steady_ns, 987654321ULL);
}
```

- [ ] **Step 2: Verify RED**

```bash
colcon test --packages-select agt_timebase --event-handlers console_direct+
```

Expected: compile failure before API exists.

- [ ] **Step 3: Implement mmap writer/reader**

Writer owns a mapped `SharedTimebaseV1`, initializes magic/version, writes stamp/update-time, then publishes a monotonically increasing sequence with release ordering. Reader snapshots the record with acquire ordering and returns `std::nullopt` for invalid magic/version or torn sequence changes.

- [ ] **Step 4: Verify GREEN**

```bash
colcon test --packages-select agt_timebase --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/agt_timebase
git commit -m "feat: add shared lidar timebase"
```

---

### Task 3: Auditable Livox timebase patch

**Files:**
- Create: `patches/livox_ros_driver2/0001-agt-shared-timebase.patch`
- Create: `scripts/apply_livox_patch.sh`
- Modify: `docs/TIME_SYNC.md`, `README.md`

**Interfaces:** Pinned upstream CustomMsg path writes the exact `pkg.base_time` to `SharedTimebaseWriter` immediately before publication; it does not reconstruct time from ROS arrival.

- [ ] **Step 1: Verify patch helper initially fails**

```bash
rm -rf /tmp/agt_livox_patch_test
git clone https://github.com/Livox-SDK/livox_ros_driver2.git /tmp/agt_livox_patch_test
git -C /tmp/agt_livox_patch_test checkout 4a1def929e5b59c7a8122d19fce6efba581ce9f7
scripts/apply_livox_patch.sh /tmp/agt_livox_patch_test
```

- [ ] **Step 2: Implement the minimal patch**

Patch only build/package dependency plus one `SharedTimebaseWriter` and this operation before CustomMsg publish:

```cpp
shared_timebase_writer_->write(
    static_cast<uint64_t>(pkg.base_time),
    std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
```

Do not alter Livox message stamps and do not add a ROS subscription.

- [ ] **Step 3: Add idempotent apply helper**

```bash
#!/usr/bin/env bash
set -euo pipefail
repo="${1:?usage: apply_livox_patch.sh PATH_TO_LIVOX_ROS_DRIVER2}"
patch_file="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/patches/livox_ros_driver2/0001-agt-shared-timebase.patch"
if git -C "$repo" apply --check "$patch_file"; then
  git -C "$repo" apply "$patch_file"
elif git -C "$repo" apply --reverse --check "$patch_file"; then
  echo "AGT Livox timebase patch already applied"
else
  echo "AGT Livox timebase patch does not apply cleanly" >&2
  exit 2
fi
```

- [ ] **Step 4: Verify two-pass apply and build**

```bash
scripts/apply_livox_patch.sh /tmp/agt_livox_patch_test
scripts/apply_livox_patch.sh /tmp/agt_livox_patch_test
vcs import src < dependencies.repos
scripts/apply_livox_patch.sh src/livox_ros_driver2
colcon build --packages-select agt_timebase livox_ros_driver2 --event-handlers console_direct+
```

Expected: second patch application reports already applied; build succeeds.

- [ ] **Step 5: Commit**

```bash
git add patches scripts docs/TIME_SYNC.md README.md
git commit -m "feat: expose livox hardware timebase"
```

---

### Task 4: Pure read-only G70 UBX NAV-PVT parser

**Files:**
- Create: `src/agt_g70_driver/{CMakeLists.txt,package.xml}`
- Create: `src/agt_g70_driver/include/agt_g70_driver/ubx.hpp`
- Create: `src/agt_g70_driver/src/ubx.cpp`
- Create: `src/agt_g70_driver/test/test_ubx.cpp`

**Interfaces:**

```cpp
std::vector<UbxFrame> UbxParser::push(std::span<const uint8_t> bytes);
std::optional<NavPvt> decode_nav_pvt(const UbxFrame & frame);
```

Accept NAV-PVT `class=0x01`, `id=0x07`, payload length 92.

- [ ] **Step 1: Write failing fixture tests**

Build a deterministic 92-byte little-endian NAV-PVT fixture in the test. Verify iTOW=345000, UTC date 2026-08-16, valid date/time/fully-resolved, carrier solution fixed, latitude `23.1581234`, longitude `113.3456789`, and hAcc `0.012 m`. Also test one-byte fragmentation, noise before sync, bad checksum, non-NAV-PVT and invalid payload length.

- [ ] **Step 2: Verify RED**

```bash
colcon test --packages-select agt_g70_driver --event-handlers console_direct+
```

- [ ] **Step 3: Implement framing/checksum**

Checksum is UBX 8-bit Fletcher over `CLASS ID LENGTH_L LENGTH_H PAYLOAD`:

```cpp
ck_a = static_cast<uint8_t>(ck_a + byte);
ck_b = static_cast<uint8_t>(ck_b + ck_a);
```

Bound payload length to 1024 bytes and resynchronize after malformed input.

- [ ] **Step 4: Decode units exactly**

```text
lat/lon: int32 * 1e-7 deg
height/hMSL/hAcc/vAcc: mm -> m
velN/E/D/gSpeed/sAcc: mm/s -> m/s
headMot/headAcc: 1e-5 deg -> rad
carrier flags bits 6-7: 0 none, 1 float, 2 fixed
```

- [ ] **Step 5: Verify GREEN and commit**

```bash
colcon test --packages-select agt_g70_driver --event-handlers console_direct+
colcon test-result --verbose
git add src/agt_g70_driver
git commit -m "feat: parse G70 UBX NAV-PVT"
```

---

### Task 5: G70 serial reader and canonical ROS publishers

**Files:**
- Create: `src/agt_g70_driver/include/agt_g70_driver/serial_port.hpp`
- Create: `src/agt_g70_driver/src/serial_port.cpp`
- Create: `src/agt_g70_driver/src/g70_node.cpp`
- Create: `src/agt_g70_driver/config/g70.yaml`
- Modify: G70 build files and `docs/G70_PROTOCOL.md`

**Interfaces:** Produces `/agt/sensors/gnss/fix`, `/velocity`, `/time_reference`, `/status`.

- [ ] **Step 1: Add conversion helper and tests**

```cpp
struct RosGnssSample {
  sensor_msgs::msg::NavSatFix fix;
  geometry_msgs::msg::TwistWithCovarianceStamped velocity;
  sensor_msgs::msg::TimeReference time_reference;
  agt_capture_msgs::msg::GnssStatus status;
};
std::optional<RosGnssSample> to_ros_sample(
    const NavPvt &, const rclcpp::Time & host_receive_time,
    const std::string & frame_id);
```

Tests: valid UTC uses GNSS sensor time; invalid/not-fully-resolved UTC sets `status.time_valid=false` and standard message stamps use host receive time rather than fabricated GNSS UTC.

- [ ] **Step 2: Implement read-only serial API**

Public API may open/configure/read/poll/close only. Do not expose or call `write()` on the GNSS descriptor.

- [ ] **Step 3: Implement UTC/covariance semantics**

Use `timegm()` plus signed `nano`; require valid-date, valid-time and fully-resolved. Position covariance diagonal is `hAcc², hAcc², vAcc²`; velocity X/Y/Z covariance diagonal uses `sAcc²`.

- [ ] **Step 4: Build/test and missing-device smoke test**

```bash
colcon build --packages-select agt_capture_msgs agt_g70_driver --event-handlers console_direct+
colcon test --packages-select agt_g70_driver --event-handlers console_direct+
ros2 run agt_g70_driver g70_node --ros-args -p port:=/dev/definitely_missing_g70
```

Expected: tests pass; missing device produces clear non-zero startup failure without receiver writes.

- [ ] **Step 5: Commit**

```bash
git add src/agt_g70_driver docs/G70_PROTOCOL.md
git commit -m "feat: publish canonical G70 GNSS data"
```

---

### Task 6: Minimal Hikrobot Line0 camera driver

**Files:**
- Create: `src/agt_hik_camera_driver/{CMakeLists.txt,package.xml}`
- Create: `src/agt_hik_camera_driver/src/camera_node.cpp`
- Create: `src/agt_hik_camera_driver/config/camera.yaml`
- Modify: `docs/HARDWARE.md`, `docs/TIME_SYNC.md`

**Interfaces:** Consumes MVS SDK + `SharedTimebaseReader`; produces image, camera_info and `/agt/sensors/camera/front/status`.

- [ ] **Step 1: Write pure timestamp-decision tests**

```cpp
struct CameraStampDecision {
  uint64_t stamp_ns;
  bool timebase_valid;
  bool stale;
  bool repeated;
};
CameraStampDecision choose_camera_stamp(
    const std::optional<agt_timebase::SharedTimebaseV1> & record,
    uint64_t previous_sequence, uint64_t host_steady_now_ns,
    uint64_t stale_threshold_ns, uint64_t fallback_ros_now_ns);
```

Prove fresh record uses LiDAR stamp; repeated sequence marks repeated but does not request a drop; stale marks stale; invalid/no record falls back to ROS now with `timebase_valid=false`.

- [ ] **Step 2: Configure MVS in fixed order**

```text
TriggerMode Off -> TriggerSource Line0 -> TriggerActivation FallingEdge
-> TriggerDelay 0 -> AcquisitionFrameRateEnable false
-> ExposureAuto Off -> ExposureTime 5000 us
-> GainAuto Continuous -> PixelFormat RGB8Packed -> TriggerMode On
```

Every required SDK setting failure aborts startup.

- [ ] **Step 3: Implement non-waiting capture**

Use bounded `MV_CC_GetImageBuffer`, snapshot shared timebase, convert/copy/publish, `MV_CC_FreeImageBuffer`, immediately continue. No LiDAR ROS subscription and no condition-variable wait for synchronization.

- [ ] **Step 4: Track evidence counters**

SDK receive/timeout, conversion failure, publish count, frame-number gaps, timestamp rollback, stale/repeated timebase, latest frame number and trigger index when SDK supplies it.

- [ ] **Step 5: Build and 60-second hardware smoke test**

```bash
colcon build --packages-select agt_capture_msgs agt_timebase agt_hik_camera_driver --event-handlers console_direct+
ros2 topic hz /agt/sensors/camera/front/image_raw
ros2 topic echo /agt/sensors/camera/front/status --once
```

Expected: near 10 Hz, no timestamp rollback, no sustained frame gaps, and no synchronization-drop mechanism.

- [ ] **Step 6: Commit**

```bash
git add src/agt_hik_camera_driver docs/HARDWARE.md docs/TIME_SYNC.md
git commit -m "feat: add Line0 Hikrobot capture driver"
```

---

### Task 7: Observer-only capture monitor

**Files:**
- Create: `src/agt_capture_monitor/{CMakeLists.txt,package.xml}`
- Create: `src/agt_capture_monitor/src/monitor_node.cpp`

**Interfaces:** Consumes canonical sensor/status topics; publishes `/agt/sensors/sync/status` and `/diagnostics`; never republishes/mutates sensor data.

- [ ] **Step 1: Test per-stream tracker**

```cpp
class StreamTracker {
public:
  void observe(int64_t stamp_ns, std::chrono::steady_clock::time_point receive_time);
  double rate_hz() const;
  uint64_t rollback_count() const;
};
```

Test synthetic 10 Hz, 5 Hz, 200 Hz and one rollback.

- [ ] **Step 2: Implement live thresholds**

```text
camera WARN <9.5 Hz; LiDAR live range 9.5-10.5 Hz
IMU live range 180-220 Hz; GNSS live range 4.5-5.5 Hz
rollback => ERROR
```

- [ ] **Step 3: Publish explicit claims**

```text
lidar_camera_sync=hardware_trigger_driver_timebase
gnss_sync=independent_gnss_clock_no_pps_bridge
```

- [ ] **Step 4: Build/test/commit**

```bash
colcon build --packages-select agt_capture_monitor --event-handlers console_direct+
colcon test --packages-select agt_capture_monitor --event-handlers console_direct+
colcon test-result --verbose
git add src/agt_capture_monitor
git commit -m "feat: add acquisition health monitor"
```

---

### Task 8: Bringup, rosbag2 recording, and inspection tools

**Files:**
- Create: `src/agt_capture_bringup/{CMakeLists.txt,package.xml}`
- Create: `src/agt_capture_bringup/launch/sensors.launch.py`
- Create: `src/agt_capture_bringup/launch/record.launch.py`
- Create: `src/agt_capture_bringup/config/record_topics.yaml`
- Create: `tools/check_rates.py`, `tools/check_timestamps.py`, `tools/inspect_bag.py`
- Modify: `test/test_topic_contract.py`, `README.md`

**Interfaces:** One-command launch and canonical rosbag2 session.

- [ ] **Step 1: Extend static test to recording topics**

Assert the recording list contains canonical v0.1 topics plus `/diagnostics`, `/tf`, `/tf_static`, and contains no `/livox/*` or `/left_camera/*` vendor topic.

- [ ] **Step 2: Implement sensor launch graph**

```text
livox_ros_driver2 with remaps:
  /livox/lidar -> /agt/sensors/lidar/custom
  /livox/imu   -> /agt/sensors/imu/data
agt_hik_camera_driver
agt_g70_driver
agt_capture_monitor
```

Expose `g70_port`, `g70_baud`, `camera_params`, `livox_config`, `timebase_path` arguments.

- [ ] **Step 3: Implement session metadata and recorder**

Before `ros2 bag record --storage sqlite3`, write:

```yaml
sync:
  lidar_camera: hardware_trigger_driver_timebase
  gnss: independent_gnss_clock_no_pps_bridge
expected_rates_hz:
  lidar: 10.0
  imu: 200.0
  camera: 10.0
  gnss: 5.0
```

- [ ] **Step 4: Implement tools with non-zero failure exits**

`check_rates.py`: observed rates over a duration. `check_timestamps.py`: count/rollback/duplicate/min/max delta. `inspect_bag.py`: parse `ros2 bag info --yaml`, verify required topics and expected count/rate ranges, exit 2 on acceptance failure.

- [ ] **Step 5: Full static build/test**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q test/test_topic_contract.py
colcon build --symlink-install --event-handlers console_direct+
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

- [ ] **Step 6: Commit**

```bash
git add src/agt_capture_bringup tools test README.md
git commit -m "feat: add unified sensor bringup and recording"
```

---

### Task 9: Hardware acceptance evidence

**Files:**
- Create: `docs/acceptance/v0.1-static.md`
- Create: `docs/acceptance/v0.1-motion.md`
- Modify: `README.md`

- [ ] **Step 1: Five-minute static run**

```bash
ros2 launch agt_capture_bringup sensors.launch.py
ros2 launch agt_capture_bringup record.launch.py session_name:=v01_static
python3 tools/check_rates.py --duration 300
python3 tools/check_timestamps.py --duration 300
```

Required: camera 9.9-10.1 Hz average, sustained camera frame gaps 0, camera rollback 0, LiDAR about 10 Hz, IMU about 200 Hz, LiDAR/IMU rollback 0, GNSS about 5 Hz, healthy-wired UBX checksum failures 0, GNSS rollback 0.

- [ ] **Step 2: Five-minute motion run with straights and turns**

```bash
ros2 launch agt_capture_bringup record.launch.py session_name:=v01_motion
```

Record GNSS carrier-state continuity and all timestamp/frame-gap evidence through motion.

- [ ] **Step 3: Inspect both sessions**

```bash
python3 tools/inspect_bag.py datasets/v01_static
python3 tools/inspect_bag.py datasets/v01_motion
```

Expected: both exit 0 before v0.1 acceptance.

- [ ] **Step 4: Commit evidence**

```bash
git add docs/acceptance README.md
git commit -m "test: record v0.1 sensor capture acceptance"
```

---

## Plan Self-Review

- Spec coverage: repository/license, topic contract, LiDAR timebase, no host-arrival pairing, no sync-induced camera drops, read-only G70 NAV-PVT, GNSS sensor/host time separation, RTK fixed/float, observer-only diagnostics, rosbag sync claim, static/motion acceptance are each mapped to Tasks 1-9.
- Placeholder scan: no unresolved implementation placeholders are present; hardware-only checks are isolated to explicit acceptance steps.
- Type consistency: `GnssStatus`, `CameraStatus`, `SharedTimebaseV1`, `UbxParser`, `NavPvt`, `RosGnssSample`, `CameraStampDecision`, and `StreamTracker` have one definition and consistent downstream use.
