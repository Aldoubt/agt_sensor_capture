#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <agt_capture_msgs/msg/camera_status.hpp>

#include "MvCameraControl.h"
#include "agt_hik_camera_driver/timestamp_policy.hpp"
#include "agt_timebase/shared_timebase.hpp"

using namespace std::chrono_literals;

namespace agt_hik_camera_driver
{
namespace
{
uint64_t steady_now_ns()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::string camera_serial(const MV_CC_DEVICE_INFO * info)
{
  if (info == nullptr) return {};
  if (info->nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  return {};
}
}  // namespace

class CameraNode final : public rclcpp::Node
{
public:
  CameraNode()
  : Node("agt_hik_camera_driver"), system_clock_(RCL_SYSTEM_TIME)
  {
    serial_number_ = declare_parameter<std::string>("serial_number", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_front_optical_frame");
    camera_name_ = declare_parameter<std::string>("camera_name", "front_camera");
    camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");
    trigger_source_ = declare_parameter<std::string>("trigger_source", "Line0");
    trigger_activation_ = declare_parameter<std::string>("trigger_activation", "FallingEdge");
    trigger_delay_us_ = declare_parameter<double>("trigger_delay_us", 0.0);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "RGB8Packed");
    exposure_auto_ = declare_parameter<std::string>("exposure_auto", "Off");
    exposure_time_us_ = declare_parameter<double>("exposure_time_us", 5000.0);
    gain_auto_ = declare_parameter<std::string>("gain_auto", "Continuous");
    expected_rate_hz_ = declare_parameter<double>("expected_rate_hz", 10.0);
    timebase_path_ = declare_parameter<std::string>("timebase_path", agt_timebase::kDefaultPath);
    const auto stale_ms = declare_parameter<int>("timebase_stale_threshold_ms", 250);
    get_image_timeout_ms_ = declare_parameter<int>("get_image_timeout_ms", 1000);
    if (stale_ms <= 0 || get_image_timeout_ms_ <= 0 || expected_rate_hz_ <= 0.0) {
      throw std::runtime_error("camera timing parameters must be positive");
    }
    stale_threshold_ns_ = static_cast<uint64_t>(stale_ms) * 1000000ULL;

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/agt/sensors/camera/front/image_raw", rclcpp::SensorDataQoS());
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/agt/sensors/camera/front/camera_info", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<agt_capture_msgs::msg::CameraStatus>(
      "/agt/sensors/camera/front/status", 10);

    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    if (!camera_info_url_.empty()) {
      if (!camera_info_manager_->validateURL(camera_info_url_) ||
          !camera_info_manager_->loadCameraInfo(camera_info_url_))
      {
        throw std::runtime_error("failed to load camera_info_url: " + camera_info_url_);
      }
    }

    if (!open_camera() || !configure_camera() || !start_camera()) {
      cleanup_camera();
      throw std::runtime_error("failed to initialize Hikrobot camera");
    }

    running_.store(true);
    capture_thread_ = std::thread(&CameraNode::capture_loop, this);
    status_timer_ = create_wall_timer(1s, std::bind(&CameraNode::publish_status, this));
    rate_window_start_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(), "Hikrobot Line0 capture started, expected %.1f Hz, timebase=%s",
      expected_rate_hz_, timebase_path_.c_str());
  }

  ~CameraNode() override
  {
    running_.store(false);
    if (capture_thread_.joinable()) capture_thread_.join();
    cleanup_camera();
  }

private:
  bool open_camera()
  {
    MV_CC_DEVICE_INFO_LIST devices{};
    int status = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
    if (status != MV_OK || devices.nDeviceNum == 0) {
      RCLCPP_ERROR(get_logger(), "No Hikrobot camera found, status=0x%x", status);
      return false;
    }

    MV_CC_DEVICE_INFO * selected = nullptr;
    for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
      const auto serial = camera_serial(devices.pDeviceInfo[i]);
      if (serial_number_.empty() || serial == serial_number_) {
        selected = devices.pDeviceInfo[i];
        RCLCPP_INFO(get_logger(), "Selected camera serial: %s", serial.c_str());
        break;
      }
    }
    if (selected == nullptr) {
      RCLCPP_ERROR(get_logger(), "Requested camera serial '%s' not found", serial_number_.c_str());
      return false;
    }

    status = MV_CC_CreateHandle(&camera_handle_, selected);
    if (status != MV_OK) {
      RCLCPP_ERROR(get_logger(), "MV_CC_CreateHandle failed: 0x%x", status);
      camera_handle_ = nullptr;
      return false;
    }
    status = MV_CC_OpenDevice(camera_handle_);
    if (status != MV_OK) {
      RCLCPP_ERROR(get_logger(), "MV_CC_OpenDevice failed: 0x%x", status);
      return false;
    }
    camera_open_ = true;
    return true;
  }

  bool set_bool(const char * key, bool value, bool required = true)
  {
    const int status = MV_CC_SetBoolValue(camera_handle_, key, value);
    if (status == MV_OK) return true;
    if (required) RCLCPP_ERROR(get_logger(), "Failed to set %s=%s: 0x%x", key, value ? "true" : "false", status);
    else RCLCPP_WARN(get_logger(), "Optional setting %s=%s failed: 0x%x", key, value ? "true" : "false", status);
    return !required;
  }

  bool set_enum(const char * key, const std::string & value, bool required = true)
  {
    const int status = MV_CC_SetEnumValueByString(camera_handle_, key, value.c_str());
    if (status == MV_OK) return true;
    if (required) RCLCPP_ERROR(get_logger(), "Failed to set %s=%s: 0x%x", key, value.c_str(), status);
    else RCLCPP_WARN(get_logger(), "Optional setting %s=%s failed: 0x%x", key, value.c_str(), status);
    return !required;
  }

  bool set_float(const char * key, double value, bool required = true)
  {
    const int status = MV_CC_SetFloatValue(camera_handle_, key, static_cast<float>(value));
    if (status == MV_OK) return true;
    if (required) RCLCPP_ERROR(get_logger(), "Failed to set %s=%.3f: 0x%x", key, value, status);
    else RCLCPP_WARN(get_logger(), "Optional setting %s=%.3f failed: 0x%x", key, value, status);
    return !required;
  }

  bool configure_camera()
  {
    if (pixel_format_ != "RGB8Packed") {
      RCLCPP_ERROR(get_logger(), "v0.1 requires pixel_format=RGB8Packed");
      return false;
    }
    bool ok = true;
    ok = set_enum("TriggerMode", "Off") && ok;
    ok = set_enum("TriggerSource", trigger_source_) && ok;
    ok = set_enum("TriggerActivation", trigger_activation_) && ok;
    ok = set_float("TriggerDelay", trigger_delay_us_, false) && ok;
    ok = set_bool("AcquisitionFrameRateEnable", false) && ok;
    ok = set_enum("ExposureAuto", exposure_auto_) && ok;
    if (exposure_auto_ == "Off") ok = set_float("ExposureTime", exposure_time_us_) && ok;
    ok = set_enum("GainAuto", gain_auto_) && ok;
    ok = set_enum("PixelFormat", pixel_format_) && ok;
    ok = set_enum("TriggerMode", "On") && ok;
    return ok;
  }

  bool start_camera()
  {
    const int status = MV_CC_StartGrabbing(camera_handle_);
    if (status != MV_OK) {
      RCLCPP_ERROR(get_logger(), "MV_CC_StartGrabbing failed: 0x%x", status);
      return false;
    }
    grabbing_ = true;
    return true;
  }

  std::optional<agt_timebase::SharedTimebaseV1> read_timebase()
  {
    if (!timebase_reader_ || !timebase_reader_->valid()) {
      timebase_reader_ = std::make_unique<agt_timebase::SharedTimebaseReader>(timebase_path_);
    }
    return timebase_reader_->read();
  }

  void capture_loop()
  {
    uint64_t previous_frame_number = 0;
    uint64_t previous_stamp_ns = 0;
    uint64_t previous_sequence = 0;

    while (rclcpp::ok() && running_.load()) {
      MV_FRAME_OUT frame{};
      const int status = MV_CC_GetImageBuffer(camera_handle_, &frame, get_image_timeout_ms_);
      if (status != MV_OK) {
        if (status == MV_E_NODATA || status == MV_E_GC_TIMEOUT) {
          ++sdk_timeout_count_;
          continue;
        }
        ++sdk_timeout_count_;
        RCLCPP_WARN(get_logger(), "MV_CC_GetImageBuffer failed: 0x%x", status);
        continue;
      }
      ++sdk_receive_count_;

      const uint64_t frame_number = frame.stFrameInfo.nFrameNum;
      if (previous_frame_number != 0 && frame_number > previous_frame_number + 1) {
        frame_gap_count_.fetch_add(frame_number - previous_frame_number - 1);
      }
      previous_frame_number = frame_number;
      last_frame_number_.store(frame_number);

      const uint64_t steady_ns = steady_now_ns();
      const uint64_t fallback_ns = static_cast<uint64_t>(system_clock_.now().nanoseconds());
      const auto decision = choose_camera_stamp(
        read_timebase(), previous_sequence, steady_ns, stale_threshold_ns_, fallback_ns);
      if (decision.timebase_valid) previous_sequence = decision.sequence;
      if (decision.repeated) ++repeated_timebase_count_;
      if (decision.stale) ++stale_timebase_count_;
      timebase_valid_.store(decision.timebase_valid);
      if (previous_stamp_ns != 0 && decision.stamp_ns < previous_stamp_ns) {
        ++timestamp_rollback_count_;
      }
      previous_stamp_ns = decision.stamp_ns;

      const uint32_t width = frame.stFrameInfo.nWidth;
      const uint32_t height = frame.stFrameInfo.nHeight;
      const std::size_t expected_size = static_cast<std::size_t>(width) * height * 3U;
      if (frame.stFrameInfo.enPixelType != PixelType_Gvsp_RGB8_Packed ||
          frame.pBufAddr == nullptr || frame.stFrameInfo.nFrameLen < expected_size)
      {
        ++conversion_failure_count_;
        MV_CC_FreeImageBuffer(camera_handle_, &frame);
        continue;
      }

      sensor_msgs::msg::Image image;
      image.header.stamp = rclcpp::Time(static_cast<int64_t>(decision.stamp_ns), RCL_SYSTEM_TIME).to_msg();
      image.header.frame_id = frame_id_;
      image.height = height;
      image.width = width;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.is_bigendian = false;
      image.step = width * 3U;
      image.data.assign(frame.pBufAddr, frame.pBufAddr + expected_size);

      auto camera_info = camera_info_manager_->getCameraInfo();
      camera_info.header = image.header;
      if (camera_info.width == 0) camera_info.width = width;
      if (camera_info.height == 0) camera_info.height = height;

      MV_CC_FreeImageBuffer(camera_handle_, &frame);
      image_pub_->publish(image);
      camera_info_pub_->publish(camera_info);
      ++publish_count_;
    }
  }

  void publish_status()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - rate_window_start_).count();
    const uint64_t sdk_count = sdk_receive_count_.load();
    const uint64_t pub_count = publish_count_.load();
    double sdk_rate = 0.0;
    double publish_rate = 0.0;
    if (elapsed > 0.0) {
      sdk_rate = static_cast<double>(sdk_count - rate_sdk_start_) / elapsed;
      publish_rate = static_cast<double>(pub_count - rate_publish_start_) / elapsed;
    }
    rate_window_start_ = now;
    rate_sdk_start_ = sdk_count;
    rate_publish_start_ = pub_count;

    agt_capture_msgs::msg::CameraStatus status;
    status.header.stamp = system_clock_.now().to_msg();
    status.header.frame_id = frame_id_;
    status.sdk_receive_count = sdk_count;
    status.sdk_timeout_count = sdk_timeout_count_.load();
    status.conversion_failure_count = conversion_failure_count_.load();
    status.publish_count = pub_count;
    status.frame_gap_count = frame_gap_count_.load();
    status.timestamp_rollback_count = timestamp_rollback_count_.load();
    status.stale_timebase_count = stale_timebase_count_.load();
    status.repeated_timebase_count = repeated_timebase_count_.load();
    status.last_frame_number = last_frame_number_.load();
    status.last_trigger_index = 0;
    status.sdk_rate_hz = sdk_rate;
    status.publish_rate_hz = publish_rate;
    status.timebase_valid = timebase_valid_.load();
    status_pub_->publish(status);
  }

  void cleanup_camera() noexcept
  {
    if (camera_handle_ != nullptr && grabbing_) {
      MV_CC_StopGrabbing(camera_handle_);
      grabbing_ = false;
    }
    if (camera_handle_ != nullptr && camera_open_) {
      MV_CC_CloseDevice(camera_handle_);
      camera_open_ = false;
    }
    if (camera_handle_ != nullptr) {
      MV_CC_DestroyHandle(camera_handle_);
      camera_handle_ = nullptr;
    }
  }

  rclcpp::Clock system_clock_;
  std::string serial_number_;
  std::string frame_id_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string trigger_source_;
  std::string trigger_activation_;
  std::string pixel_format_;
  std::string exposure_auto_;
  std::string gain_auto_;
  std::string timebase_path_;
  double trigger_delay_us_{0.0};
  double exposure_time_us_{5000.0};
  double expected_rate_hz_{10.0};
  int get_image_timeout_ms_{1000};
  uint64_t stale_threshold_ns_{250000000ULL};

  void * camera_handle_{nullptr};
  bool camera_open_{false};
  bool grabbing_{false};
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::unique_ptr<agt_timebase::SharedTimebaseReader> timebase_reader_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<agt_capture_msgs::msg::CameraStatus>::SharedPtr status_pub_;

  std::atomic<uint64_t> sdk_receive_count_{0};
  std::atomic<uint64_t> sdk_timeout_count_{0};
  std::atomic<uint64_t> conversion_failure_count_{0};
  std::atomic<uint64_t> publish_count_{0};
  std::atomic<uint64_t> frame_gap_count_{0};
  std::atomic<uint64_t> timestamp_rollback_count_{0};
  std::atomic<uint64_t> stale_timebase_count_{0};
  std::atomic<uint64_t> repeated_timebase_count_{0};
  std::atomic<uint64_t> last_frame_number_{0};
  std::atomic<bool> timebase_valid_{false};

  std::chrono::steady_clock::time_point rate_window_start_{};
  uint64_t rate_sdk_start_{0};
  uint64_t rate_publish_start_{0};
};

}  // namespace agt_hik_camera_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<agt_hik_camera_driver::CameraNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("agt_hik_camera_driver"), "%s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
