#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "agt_g70_driver/ros_conversion.hpp"
#include "agt_g70_driver/serial_port.hpp"
#include "agt_g70_driver/ubx.hpp"

using namespace std::chrono_literals;

namespace agt_g70_driver
{
namespace
{
diagnostic_msgs::msg::KeyValue kv(std::string key, uint64_t value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::to_string(value);
  return item;
}
}  // namespace

class G70Node final : public rclcpp::Node
{
public:
  G70Node()
  : Node("agt_g70_driver"), system_clock_(RCL_SYSTEM_TIME)
  {
    const auto port = declare_parameter<std::string>("port", "/dev/wheeltec_gnss");
    const auto baudrate = declare_parameter<int>("baudrate", 9600);
    frame_id_ = declare_parameter<std::string>("frame_id", "gnss_link");
    read_timeout_ms_ = declare_parameter<int>("read_timeout_ms", 200);
    expected_rate_hz_ = declare_parameter<double>("expected_rate_hz", 5.0);

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/agt/sensors/gnss/fix", 20);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/agt/sensors/gnss/velocity", 20);
    time_reference_pub_ = create_publisher<sensor_msgs::msg::TimeReference>(
      "/agt/sensors/gnss/time_reference", 20);
    status_pub_ = create_publisher<agt_capture_msgs::msg::GnssStatus>(
      "/agt/sensors/gnss/status", 20);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    serial_ = std::make_unique<ReadOnlySerialPort>(port, baudrate);
    if (!serial_->open()) {
      throw std::runtime_error("failed to open G70 " + port + ": " + serial_->last_error());
    }

    running_.store(true);
    worker_ = std::thread(&G70Node::read_loop, this);
    diagnostic_timer_ = create_wall_timer(1s, std::bind(&G70Node::publish_diagnostics, this));
    RCLCPP_INFO(
      get_logger(), "G70 read-only UBX NAV-PVT acquisition started: %s @ %d baud, expected %.1f Hz",
      port.c_str(), baudrate, expected_rate_hz_);
  }

  ~G70Node() override
  {
    running_.store(false);
    if (serial_) serial_->close();
    if (worker_.joinable()) worker_.join();
  }

private:
  void read_loop()
  {
    std::array<uint8_t, 2048> buffer{};
    while (rclcpp::ok() && running_.load()) {
      const ssize_t count = serial_->read_some(buffer.data(), buffer.size(), read_timeout_ms_);
      if (count == 0) continue;
      if (count < 0) {
        ++read_error_count_;
        RCLCPP_ERROR(get_logger(), "G70 serial read stopped: %s", serial_->last_error().c_str());
        break;
      }

      const rclcpp::Time host_receive_time = system_clock_.now();
      const auto frames = parser_.push(buffer.data(), static_cast<std::size_t>(count));
      checksum_failure_count_.store(parser_.checksum_failure_count());
      oversize_frame_count_.store(parser_.oversize_frame_count());
      for (const auto & frame : frames) {
        auto nav = decode_nav_pvt(frame);
        if (!nav.has_value()) {
          ++ignored_ubx_count_;
          continue;
        }
        auto sample = to_ros_sample(*nav, host_receive_time, frame_id_);
        if (!sample.has_value()) {
          ++invalid_nav_pvt_count_;
          continue;
        }
        fix_pub_->publish(sample->fix);
        velocity_pub_->publish(sample->velocity);
        time_reference_pub_->publish(sample->time_reference);
        status_pub_->publish(sample->status);
        ++nav_pvt_count_;
      }
    }
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = system_clock_.now().to_msg();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "agt_g70_driver";
    status.hardware_id = "WHEELTEC_G70";
    if (read_error_count_.load() > 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial read error";
    } else if (checksum_failure_count_.load() > 0 || oversize_frame_count_.load() > 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "UBX framing errors observed";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "read-only UBX NAV-PVT acquisition active";
    }
    status.values.push_back(kv("nav_pvt_count", nav_pvt_count_.load()));
    status.values.push_back(kv("checksum_failure_count", checksum_failure_count_.load()));
    status.values.push_back(kv("oversize_frame_count", oversize_frame_count_.load()));
    status.values.push_back(kv("ignored_ubx_count", ignored_ubx_count_.load()));
    status.values.push_back(kv("invalid_nav_pvt_count", invalid_nav_pvt_count_.load()));
    status.values.push_back(kv("read_error_count", read_error_count_.load()));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  rclcpp::Clock system_clock_;
  std::string frame_id_;
  int read_timeout_ms_{200};
  double expected_rate_hz_{5.0};
  std::unique_ptr<ReadOnlySerialPort> serial_;
  UbxParser parser_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_reference_pub_;
  rclcpp::Publisher<agt_capture_msgs::msg::GnssStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  std::atomic<uint64_t> nav_pvt_count_{0};
  std::atomic<uint64_t> checksum_failure_count_{0};
  std::atomic<uint64_t> oversize_frame_count_{0};
  std::atomic<uint64_t> ignored_ubx_count_{0};
  std::atomic<uint64_t> invalid_nav_pvt_count_{0};
  std::atomic<uint64_t> read_error_count_{0};
};

}  // namespace agt_g70_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<agt_g70_driver::G70Node>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("agt_g70_driver"), "%s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
