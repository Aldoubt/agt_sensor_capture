#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <agt_capture_msgs/msg/camera_status.hpp>
#include <agt_capture_msgs/msg/gnss_status.hpp>

#include "agt_capture_monitor/stream_tracker.hpp"

using namespace std::chrono_literals;

namespace agt_capture_monitor
{
namespace
{

int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
    static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time to_time_msg(const rclcpp::Time & time)
{
  return static_cast<builtin_interfaces::msg::Time>(time);
}

diagnostic_msgs::msg::KeyValue kv(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

diagnostic_msgs::msg::KeyValue kv(std::string key, double value)
{
  return kv(std::move(key), std::to_string(value));
}

diagnostic_msgs::msg::KeyValue kv(std::string key, uint64_t value)
{
  return kv(std::move(key), std::to_string(value));
}

uint8_t rate_level(double rate, double min_rate, double max_rate, uint64_t samples)
{
  if (samples < 2U) return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  if (rate < min_rate || rate > max_rate) return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

}  // namespace

class CaptureMonitor final : public rclcpp::Node
{
public:
  CaptureMonitor()
  : Node("agt_capture_monitor"),
    lidar_(2s), imu_(1s), gnss_(3s)
  {
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/agt/sensors/lidar/custom", rclcpp::SensorDataQoS(),
      std::bind(&CaptureMonitor::on_lidar, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/agt/sensors/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&CaptureMonitor::on_imu, this, std::placeholders::_1));
    camera_sub_ = create_subscription<agt_capture_msgs::msg::CameraStatus>(
      "/agt/sensors/camera/front/status", 10,
      std::bind(&CaptureMonitor::on_camera, this, std::placeholders::_1));
    gnss_sub_ = create_subscription<agt_capture_msgs::msg::GnssStatus>(
      "/agt/sensors/gnss/status", 20,
      std::bind(&CaptureMonitor::on_gnss, this, std::placeholders::_1));

    sync_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/agt/sensors/sync/status", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    timer_ = create_wall_timer(1s, std::bind(&CaptureMonitor::publish_status, this));
  }

private:
  void on_lidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    int64_t sensor_stamp = stamp_to_ns(msg->header.stamp);
    if (msg->timebase > 0U &&
        msg->timebase <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
    {
      sensor_stamp = static_cast<int64_t>(msg->timebase);
    }
    lidar_.observe(sensor_stamp, std::chrono::steady_clock::now());
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_.observe(stamp_to_ns(msg->header.stamp), std::chrono::steady_clock::now());
  }

  void on_camera(const agt_capture_msgs::msg::CameraStatus::SharedPtr msg)
  {
    camera_status_ = *msg;
    have_camera_status_ = true;
  }

  void on_gnss(const agt_capture_msgs::msg::GnssStatus::SharedPtr msg)
  {
    gnss_.observe(stamp_to_ns(msg->header.stamp), std::chrono::steady_clock::now());
    if (!msg->time_valid) ++gnss_invalid_time_count_;
    if (msg->carrier_solution == agt_capture_msgs::msg::GnssStatus::CARRIER_FIXED) {
      ++gnss_fixed_count_;
    } else if (msg->carrier_solution == agt_capture_msgs::msg::GnssStatus::CARRIER_FLOAT) {
      ++gnss_float_count_;
    } else {
      ++gnss_no_carrier_count_;
    }
  }

  diagnostic_msgs::msg::DiagnosticStatus stream_status(
    const char * name, const StreamTracker & tracker,
    double min_rate, double max_rate) const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.hardware_id = "agt_sensor_capture";
    status.level = rate_level(tracker.rate_hz(), min_rate, max_rate, tracker.sample_count());
    status.message = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ?
      "rate healthy" : "rate outside live range or warming up";
    if (tracker.rollback_count() > 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "timestamp rollback observed";
    }
    status.values.push_back(kv("rate_hz", tracker.rate_hz()));
    status.values.push_back(kv("sample_count", tracker.sample_count()));
    status.values.push_back(kv("timestamp_rollback_count", tracker.rollback_count()));
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus camera_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "agt_capture_monitor/camera";
    status.hardware_id = "Hikrobot_front_camera";
    if (!have_camera_status_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for camera status";
      return status;
    }
    status.level = camera_status_.publish_rate_hz >= 9.5 ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ?
      "camera acquisition healthy" : "camera publish rate below 9.5 Hz";
    if (camera_status_.timestamp_rollback_count > 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "camera timestamp rollback observed";
    } else if (!camera_status_.timebase_valid || camera_status_.frame_gap_count > 0U ||
               camera_status_.stale_timebase_count > 0U || camera_status_.repeated_timebase_count > 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "camera acquisition/timing evidence degraded";
    }
    status.values.push_back(kv("sdk_rate_hz", camera_status_.sdk_rate_hz));
    status.values.push_back(kv("publish_rate_hz", camera_status_.publish_rate_hz));
    status.values.push_back(kv("sdk_receive_count", camera_status_.sdk_receive_count));
    status.values.push_back(kv("publish_count", camera_status_.publish_count));
    status.values.push_back(kv("frame_gap_count", camera_status_.frame_gap_count));
    status.values.push_back(kv("timestamp_rollback_count", camera_status_.timestamp_rollback_count));
    status.values.push_back(kv("stale_timebase_count", camera_status_.stale_timebase_count));
    status.values.push_back(kv("repeated_timebase_count", camera_status_.repeated_timebase_count));
    status.values.push_back(kv("timebase_valid", camera_status_.timebase_valid ? "true" : "false"));
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus sync_claim_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "agt_capture_monitor/sync_claim";
    status.hardware_id = "agt_sensor_capture_v0.1";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "v0.1 synchronization contract";
    status.values.push_back(kv("lidar_camera_sync", "hardware_trigger_driver_timebase"));
    status.values.push_back(kv("gnss_sync", "independent_gnss_clock_no_pps_bridge"));
    status.values.push_back(kv("gnss_fixed_count", gnss_fixed_count_));
    status.values.push_back(kv("gnss_float_count", gnss_float_count_));
    status.values.push_back(kv("gnss_no_carrier_count", gnss_no_carrier_count_));
    status.values.push_back(kv("gnss_invalid_time_count", gnss_invalid_time_count_));
    return status;
  }

  void publish_status()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = to_time_msg(now());
    array.status.push_back(stream_status("agt_capture_monitor/lidar", lidar_, 9.5, 10.5));
    array.status.push_back(stream_status("agt_capture_monitor/imu", imu_, 180.0, 220.0));
    array.status.push_back(camera_status());
    array.status.push_back(stream_status("agt_capture_monitor/gnss", gnss_, 4.5, 5.5));
    array.status.push_back(sync_claim_status());
    sync_pub_->publish(array);
    diagnostics_pub_->publish(array);
  }

  StreamTracker lidar_;
  StreamTracker imu_;
  StreamTracker gnss_;
  agt_capture_msgs::msg::CameraStatus camera_status_;
  bool have_camera_status_{false};
  uint64_t gnss_fixed_count_{0};
  uint64_t gnss_float_count_{0};
  uint64_t gnss_no_carrier_count_{0};
  uint64_t gnss_invalid_time_count_{0};

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<agt_capture_msgs::msg::CameraStatus>::SharedPtr camera_sub_;
  rclcpp::Subscription<agt_capture_msgs::msg::GnssStatus>::SharedPtr gnss_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sync_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agt_capture_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_capture_monitor::CaptureMonitor>());
  rclcpp::shutdown();
  return 0;
}
