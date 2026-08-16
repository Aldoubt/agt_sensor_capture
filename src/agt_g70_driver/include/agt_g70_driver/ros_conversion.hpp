#pragma once

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <agt_capture_msgs/msg/gnss_status.hpp>

#include "agt_g70_driver/ubx.hpp"

namespace agt_g70_driver
{

struct RosGnssSample
{
  sensor_msgs::msg::NavSatFix fix;
  geometry_msgs::msg::TwistWithCovarianceStamped velocity;
  sensor_msgs::msg::TimeReference time_reference;
  agt_capture_msgs::msg::GnssStatus status;
};

std::optional<RosGnssSample> to_ros_sample(
  const NavPvt & nav,
  const rclcpp::Time & host_receive_time,
  const std::string & frame_id);

}  // namespace agt_g70_driver
