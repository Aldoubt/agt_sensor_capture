#include <cstdint>
#include <vector>
#include <gtest/gtest.h>
#include "agt_g70_driver/ubx.hpp"

namespace
{
void put_u16(std::vector<uint8_t> & p, std::size_t o, uint16_t v)
{
  p[o] = static_cast<uint8_t>(v & 0xffU);
  p[o + 1] = static_cast<uint8_t>((v >> 8U) & 0xffU);
}
void put_u32(std::vector<uint8_t> & p, std::size_t o, uint32_t v)
{
  for (int i = 0; i < 4; ++i) p[o + i] = static_cast<uint8_t>((v >> (8 * i)) & 0xffU);
}
void put_i32(std::vector<uint8_t> & p, std::size_t o, int32_t v)
{
  put_u32(p, o, static_cast<uint32_t>(v));
}
std::vector<uint8_t> make_frame(uint8_t cls, uint8_t id, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> bytes{
    0xB5, 0x62, cls, id,
    static_cast<uint8_t>(payload.size() & 0xffU),
    static_cast<uint8_t>((payload.size() >> 8U) & 0xffU)};
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (std::size_t i = 2; i < bytes.size(); ++i) {
    ck_a = static_cast<uint8_t>(ck_a + bytes[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
  bytes.push_back(ck_a);
  bytes.push_back(ck_b);
  return bytes;
}
std::vector<uint8_t> make_nav_pvt_payload()
{
  std::vector<uint8_t> p(92, 0);
  put_u32(p, 0, 345000); put_u16(p, 4, 2026);
  p[6] = 8; p[7] = 16; p[8] = 8; p[9] = 42; p[10] = 30; p[11] = 0x07;
  put_u32(p, 12, 50); put_i32(p, 16, 123456789);
  p[20] = 3; p[21] = 0x83; p[23] = 24;
  put_i32(p, 24, 1133456789); put_i32(p, 28, 231581234);
  put_i32(p, 32, 50000); put_i32(p, 36, 30000); put_u32(p, 40, 12); put_u32(p, 44, 20);
  put_i32(p, 48, 1000); put_i32(p, 52, -500); put_i32(p, 56, 100); put_i32(p, 60, 1118);
  put_i32(p, 64, 9000000); put_u32(p, 68, 50); put_u32(p, 72, 100000);
  return p;
}
}  // namespace

TEST(UbxParser, HandlesNoiseAndOneByteFragmentation)
{
  agt_g70_driver::UbxParser parser;
  auto bytes = make_frame(0x01, 0x07, make_nav_pvt_payload());
  uint8_t noise = 0;
  EXPECT_TRUE(parser.push(&noise, 1).empty());
  std::vector<agt_g70_driver::UbxFrame> frames;
  for (const auto byte : bytes) {
    auto current = parser.push(&byte, 1);
    frames.insert(frames.end(), current.begin(), current.end());
  }
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames.front().payload.size(), 92U);
}

TEST(UbxParser, RejectsBadChecksumAndRecovers)
{
  agt_g70_driver::UbxParser parser;
  auto bad = make_frame(0x01, 0x07, make_nav_pvt_payload());
  bad.back() ^= 0x55U;
  EXPECT_TRUE(parser.push(bad).empty());
  EXPECT_EQ(parser.checksum_failure_count(), 1U);
  EXPECT_EQ(parser.push(make_frame(0x01, 0x07, make_nav_pvt_payload())).size(), 1U);
}

TEST(UbxParser, RejectsOversizeFrame)
{
  agt_g70_driver::UbxParser parser;
  const uint8_t header[]{0xB5, 0x62, 0x01, 0x07, 0x01, 0x04};
  EXPECT_TRUE(parser.push(header, sizeof(header)).empty());
  EXPECT_EQ(parser.oversize_frame_count(), 1U);
}

TEST(NavPvt, DecodesBenchmarkFieldsAndUnits)
{
  agt_g70_driver::UbxFrame frame{0x01, 0x07, make_nav_pvt_payload()};
  auto nav = agt_g70_driver::decode_nav_pvt(frame);
  ASSERT_TRUE(nav.has_value());
  EXPECT_EQ(nav->i_tow_ms, 345000U);
  EXPECT_EQ(nav->year, 2026U);
  EXPECT_TRUE(nav->time_valid);
  EXPECT_TRUE(nav->fully_resolved);
  EXPECT_TRUE(nav->gnss_fix_ok);
  EXPECT_TRUE(nav->differential_solution);
  EXPECT_EQ(nav->carrier_solution, agt_g70_driver::CarrierSolution::Fixed);
  EXPECT_NEAR(nav->latitude_deg, 23.1581234, 1e-9);
  EXPECT_NEAR(nav->longitude_deg, 113.3456789, 1e-9);
  EXPECT_NEAR(nav->horizontal_accuracy_m, 0.012, 1e-12);
  EXPECT_NEAR(nav->velocity_n_mps, 1.0, 1e-12);
  EXPECT_NEAR(nav->velocity_e_mps, -0.5, 1e-12);
  EXPECT_NEAR(nav->motion_heading_rad, 1.5707963267948966, 1e-9);
}

TEST(NavPvt, RejectsWrongMessageAndLength)
{
  auto payload = make_nav_pvt_payload();
  EXPECT_FALSE(agt_g70_driver::decode_nav_pvt({0x01, 0x02, payload}).has_value());
  payload.resize(91);
  EXPECT_FALSE(agt_g70_driver::decode_nav_pvt({0x01, 0x07, payload}).has_value());
}

TEST(NavPvtTime, RequiresValidFullyResolvedUtc)
{
  agt_g70_driver::NavPvt nav;
  nav.year = 2026; nav.month = 8; nav.day = 16;
  nav.hour = 8; nav.minute = 42; nav.second = 30;
  nav.time_valid = true; nav.fully_resolved = true; nav.nano_ns = 123456789;
  auto stamp = agt_g70_driver::nav_pvt_unix_time_ns(nav);
  ASSERT_TRUE(stamp.has_value());
  EXPECT_EQ(*stamp, 1786869750123456789LL);
  nav.fully_resolved = false;
  EXPECT_FALSE(agt_g70_driver::nav_pvt_unix_time_ns(nav).has_value());
}
