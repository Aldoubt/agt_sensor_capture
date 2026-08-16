#include "agt_g70_driver/ros_conversion.hpp"

#include <cmath>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace agt_g70_driver
{
namespace
{

builtin_interfaces::msg::Time to_time_msg(const rclcpp::Time & time)
{
  return static_cast<builtin_interfaces::msg::Time>(time);
}

}  // namespace

std::optional<RosGnssSample> to_ros_sample(
  const NavPvt & nav,
  const rclcpp::Time & host_receive_time,
  const std::string & frame_id)
{
  if (!std::isfinite(nav.latitude_deg) || !std::isfinite(nav.longitude_deg) ||
      nav.latitude_deg < -90.0 || nav.latitude_deg > 90.0 ||
      nav.longitude_deg < -180.0 || nav.longitude_deg > 180.0 ||
      nav.horizontal_accuracy_m < 0.0 || nav.vertical_accuracy_m < 0.0 ||
      nav.speed_accuracy_mps < 0.0)
  {
    return std::nullopt;
  }

  RosGnssSample sample;
  const auto sensor_time_ns = nav_pvt_unix_time_ns(nav);
  const rclcpp::Time sample_stamp = sensor_time_ns.has_value() ?
    rclcpp::Time(*sensor_time_ns, RCL_SYSTEM_TIME) : host_receive_time;

  sample.fix.header.stamp = to_time_msg(sample_stamp);
  sample.fix.header.frame_id = frame_id;
  const bool has_position = nav.gnss_fix_ok && nav.fix_type >= 2U && nav.fix_type <= 4U;
  sample.fix.status.status = has_position ?
    sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  sample.fix.status.service = 0U;
  sample.fix.latitude = nav.latitude_deg;
  sample.fix.longitude = nav.longitude_deg;
  sample.fix.altitude = nav.height_ellipsoid_m;
  if (has_position) {
    const double h_var = nav.horizontal_accuracy_m * nav.horizontal_accuracy_m;
    const double v_var = nav.vertical_accuracy_m * nav.vertical_accuracy_m;
    sample.fix.position_covariance[0] = h_var;
    sample.fix.position_covariance[4] = h_var;
    sample.fix.position_covariance[8] = v_var;
    sample.fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  } else {
    sample.fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  sample.velocity.header.stamp = to_time_msg(sample_stamp);
  sample.velocity.header.frame_id = frame_id;
  sample.velocity.twist.twist.linear.x = nav.velocity_e_mps;
  sample.velocity.twist.twist.linear.y = nav.velocity_n_mps;
  sample.velocity.twist.twist.linear.z = -nav.velocity_d_mps;
  const double speed_var = nav.speed_accuracy_mps * nav.speed_accuracy_mps;
  sample.velocity.twist.covariance[0] = speed_var;
  sample.velocity.twist.covariance[7] = speed_var;
  sample.velocity.twist.covariance[14] = speed_var;

  sample.time_reference.header.stamp = to_time_msg(host_receive_time);
  sample.time_reference.header.frame_id = frame_id;
  if (sensor_time_ns.has_value()) {
    sample.time_reference.time_ref = to_time_msg(rclcpp::Time(*sensor_time_ns, RCL_SYSTEM_TIME));
  }
  sample.time_reference.source = "WHEELTEC_G70_UBX_NAV_PVT";

  sample.status.header.stamp = to_time_msg(sample_stamp);
  sample.status.header.frame_id = frame_id;
  sample.status.host_receive_time = to_time_msg(host_receive_time);
  sample.status.i_tow_ms = nav.i_tow_ms;
  sample.status.time_accuracy_ns = nav.time_accuracy_ns;
  sample.status.time_valid = sensor_time_ns.has_value();
  sample.status.fully_resolved = nav.fully_resolved;
  sample.status.fix_type = nav.fix_type;
  sample.status.gnss_fix_ok = nav.gnss_fix_ok;
  sample.status.differential_solution = nav.differential_solution;
  switch (nav.carrier_solution) {
    case CarrierSolution::Float:
      sample.status.carrier_solution = agt_capture_msgs::msg::GnssStatus::CARRIER_FLOAT;
      break;
    case CarrierSolution::Fixed:
      sample.status.carrier_solution = agt_capture_msgs::msg::GnssStatus::CARRIER_FIXED;
      break;
    case CarrierSolution::None:
    default:
      sample.status.carrier_solution = agt_capture_msgs::msg::GnssStatus::CARRIER_NONE;
      break;
  }
  sample.status.num_satellites = nav.num_satellites;
  sample.status.horizontal_accuracy_m = nav.horizontal_accuracy_m;
  sample.status.vertical_accuracy_m = nav.vertical_accuracy_m;
  sample.status.speed_accuracy_mps = nav.speed_accuracy_mps;
  sample.status.motion_heading_rad = nav.motion_heading_rad;
  sample.status.heading_accuracy_rad = nav.heading_accuracy_rad;
  return sample;
}

}  // namespace agt_g70_driver
