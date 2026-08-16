#include "agt_g70_driver/ubx.hpp"

#include <algorithm>
#include <array>

namespace agt_g70_driver
{
namespace
{
constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;
constexpr std::size_t kHeaderSize = 6;
constexpr std::size_t kChecksumSize = 2;
constexpr std::array<uint8_t, 2> kSyncBytes{kSync1, kSync2};

bool checksum_valid(const std::vector<uint8_t> & bytes, std::size_t payload_size)
{
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  const std::size_t checksum_begin = 2;
  const std::size_t checksum_end = kHeaderSize + payload_size;
  for (std::size_t i = checksum_begin; i < checksum_end; ++i) {
    ck_a = static_cast<uint8_t>(ck_a + bytes[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
  return bytes[checksum_end] == ck_a && bytes[checksum_end + 1] == ck_b;
}

constexpr double kDegToRad = 0.01745329251994329576923690768489;

uint16_t read_u16(const std::vector<uint8_t> & payload, std::size_t offset)
{
  return static_cast<uint16_t>(payload[offset]) |
    (static_cast<uint16_t>(payload[offset + 1]) << 8U);
}

uint32_t read_u32(const std::vector<uint8_t> & payload, std::size_t offset)
{
  return static_cast<uint32_t>(payload[offset]) |
    (static_cast<uint32_t>(payload[offset + 1]) << 8U) |
    (static_cast<uint32_t>(payload[offset + 2]) << 16U) |
    (static_cast<uint32_t>(payload[offset + 3]) << 24U);
}

int32_t read_i32(const std::vector<uint8_t> & payload, std::size_t offset)
{
  const uint32_t raw = read_u32(payload, offset);
  if (raw <= 0x7fffffffU) {
    return static_cast<int32_t>(raw);
  }
  return static_cast<int32_t>(static_cast<int64_t>(raw) - 0x100000000LL);
}
}  // namespace

std::vector<UbxFrame> UbxParser::push(const uint8_t * data, std::size_t size)
{
  std::vector<UbxFrame> output;
  if (data != nullptr && size > 0) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  while (true) {
    const auto sync = std::search(
      buffer_.begin(), buffer_.end(), kSyncBytes.begin(), kSyncBytes.end());
    if (sync == buffer_.end()) {
      if (!buffer_.empty() && buffer_.back() == kSync1) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        buffer_.clear();
      }
      break;
    }
    if (sync != buffer_.begin()) {
      buffer_.erase(buffer_.begin(), sync);
    }
    if (buffer_.size() < kHeaderSize) break;

    const std::size_t payload_size =
      static_cast<std::size_t>(buffer_[4]) |
      (static_cast<std::size_t>(buffer_[5]) << 8U);
    if (payload_size > kMaxPayloadSize) {
      ++oversize_frame_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_size = kHeaderSize + payload_size + kChecksumSize;
    if (buffer_.size() < frame_size) break;
    if (!checksum_valid(buffer_, payload_size)) {
      ++checksum_failure_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    UbxFrame frame;
    frame.message_class = buffer_[2];
    frame.message_id = buffer_[3];
    frame.payload.assign(
      buffer_.begin() + kHeaderSize, buffer_.begin() + kHeaderSize + payload_size);
    output.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
  }
  return output;
}

std::optional<NavPvt> decode_nav_pvt(const UbxFrame & frame)
{
  constexpr uint8_t kNavClass = 0x01;
  constexpr uint8_t kNavPvtId = 0x07;
  constexpr std::size_t kNavPvtLength = 92;
  if (frame.message_class != kNavClass || frame.message_id != kNavPvtId ||
    frame.payload.size() != kNavPvtLength)
  {
    return std::nullopt;
  }

  const auto & p = frame.payload;
  NavPvt nav;
  nav.i_tow_ms = read_u32(p, 0);
  nav.year = read_u16(p, 4);
  nav.month = p[6];
  nav.day = p[7];
  nav.hour = p[8];
  nav.minute = p[9];
  nav.second = p[10];
  nav.valid_date = (p[11] & 0x01U) != 0;
  nav.valid_time = (p[11] & 0x02U) != 0;
  nav.time_valid = nav.valid_date && nav.valid_time;
  nav.fully_resolved = (p[11] & 0x04U) != 0;
  nav.time_accuracy_ns = read_u32(p, 12);
  nav.nano_ns = read_i32(p, 16);
  nav.fix_type = p[20];
  nav.gnss_fix_ok = (p[21] & 0x01U) != 0;
  nav.differential_solution = (p[21] & 0x02U) != 0;
  const uint8_t carrier_bits = static_cast<uint8_t>((p[21] >> 6U) & 0x03U);
  if (carrier_bits == 1U) {
    nav.carrier_solution = CarrierSolution::Float;
  } else if (carrier_bits == 2U) {
    nav.carrier_solution = CarrierSolution::Fixed;
  } else {
    nav.carrier_solution = CarrierSolution::None;
  }
  nav.num_satellites = p[23];
  nav.longitude_deg = static_cast<double>(read_i32(p, 24)) * 1e-7;
  nav.latitude_deg = static_cast<double>(read_i32(p, 28)) * 1e-7;
  nav.height_ellipsoid_m = static_cast<double>(read_i32(p, 32)) * 1e-3;
  nav.height_msl_m = static_cast<double>(read_i32(p, 36)) * 1e-3;
  nav.horizontal_accuracy_m = static_cast<double>(read_u32(p, 40)) * 1e-3;
  nav.vertical_accuracy_m = static_cast<double>(read_u32(p, 44)) * 1e-3;
  nav.velocity_n_mps = static_cast<double>(read_i32(p, 48)) * 1e-3;
  nav.velocity_e_mps = static_cast<double>(read_i32(p, 52)) * 1e-3;
  nav.velocity_d_mps = static_cast<double>(read_i32(p, 56)) * 1e-3;
  nav.ground_speed_mps = static_cast<double>(read_i32(p, 60)) * 1e-3;
  nav.motion_heading_rad = static_cast<double>(read_i32(p, 64)) * 1e-5 * kDegToRad;
  nav.speed_accuracy_mps = static_cast<double>(read_u32(p, 68)) * 1e-3;
  nav.heading_accuracy_rad = static_cast<double>(read_u32(p, 72)) * 1e-5 * kDegToRad;
  return nav;
}

}  // namespace agt_g70_driver
