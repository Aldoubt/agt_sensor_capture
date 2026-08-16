#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace agt_g70_driver
{

struct UbxFrame
{
  uint8_t message_class{0};
  uint8_t message_id{0};
  std::vector<uint8_t> payload;
};

enum class CarrierSolution : uint8_t
{
  None = 0,
  Float = 1,
  Fixed = 2,
};

struct NavPvt
{
  uint32_t i_tow_ms{0};
  uint16_t year{0};
  uint8_t month{0};
  uint8_t day{0};
  uint8_t hour{0};
  uint8_t minute{0};
  uint8_t second{0};
  bool valid_date{false};
  bool valid_time{false};
  bool time_valid{false};
  bool fully_resolved{false};
  uint32_t time_accuracy_ns{0};
  int32_t nano_ns{0};
  uint8_t fix_type{0};
  bool gnss_fix_ok{false};
  bool differential_solution{false};
  CarrierSolution carrier_solution{CarrierSolution::None};
  uint8_t num_satellites{0};
  double longitude_deg{0.0};
  double latitude_deg{0.0};
  double height_ellipsoid_m{0.0};
  double height_msl_m{0.0};
  double horizontal_accuracy_m{0.0};
  double vertical_accuracy_m{0.0};
  double velocity_n_mps{0.0};
  double velocity_e_mps{0.0};
  double velocity_d_mps{0.0};
  double ground_speed_mps{0.0};
  double motion_heading_rad{0.0};
  double speed_accuracy_mps{0.0};
  double heading_accuracy_rad{0.0};
};

std::optional<NavPvt> decode_nav_pvt(const UbxFrame & frame);
std::optional<int64_t> nav_pvt_unix_time_ns(const NavPvt & nav);

class UbxParser
{
public:
  static constexpr std::size_t kMaxPayloadSize = 1024;

  std::vector<UbxFrame> push(const uint8_t * data, std::size_t size);
  std::vector<UbxFrame> push(const std::vector<uint8_t> & bytes)
  {
    return push(bytes.data(), bytes.size());
  }

  uint64_t checksum_failure_count() const noexcept {return checksum_failure_count_;}
  uint64_t oversize_frame_count() const noexcept {return oversize_frame_count_;}

private:
  std::vector<uint8_t> buffer_;
  uint64_t checksum_failure_count_{0};
  uint64_t oversize_frame_count_{0};
};

}  // namespace agt_g70_driver
