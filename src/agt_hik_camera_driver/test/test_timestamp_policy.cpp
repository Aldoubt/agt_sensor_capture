#include <cstdint>
#include <limits>
#include <optional>
#include <gtest/gtest.h>
#include "agt_hik_camera_driver/timestamp_policy.hpp"

TEST(TimestampPolicy, UsesFreshLidarSensorTime)
{
  agt_timebase::SharedTimebaseV1 record;
  record.sequence = 10;
  record.lidar_stamp_ns = 1000000000ULL;
  record.host_update_steady_ns = 900000000ULL;
  const auto d = agt_hik_camera_driver::choose_camera_stamp(
    record, 9, 950000000ULL, 250000000ULL, 5000000000ULL);
  EXPECT_TRUE(d.timebase_valid);
  EXPECT_FALSE(d.stale);
  EXPECT_FALSE(d.repeated);
  EXPECT_EQ(d.stamp_ns, 1000000000ULL);
}

TEST(TimestampPolicy, ReportsRepeatedAndStaleWithoutDroppingTheStamp)
{
  agt_timebase::SharedTimebaseV1 record;
  record.sequence = 10;
  record.lidar_stamp_ns = 1000000000ULL;
  record.host_update_steady_ns = 900000000ULL;
  auto d = agt_hik_camera_driver::choose_camera_stamp(
    record, 10, 950000000ULL, 250000000ULL, 5000000000ULL);
  EXPECT_TRUE(d.repeated);
  EXPECT_EQ(d.stamp_ns, record.lidar_stamp_ns);
  d = agt_hik_camera_driver::choose_camera_stamp(
    record, 9, 1300000000ULL, 250000000ULL, 5000000000ULL);
  EXPECT_TRUE(d.stale);
  EXPECT_EQ(d.stamp_ns, record.lidar_stamp_ns);
}

TEST(TimestampPolicy, FallsBackOnlyWhenTimebaseIsInvalid)
{
  auto d = agt_hik_camera_driver::choose_camera_stamp(
    std::nullopt, 0, 1, 250000000ULL, 5000000000ULL);
  EXPECT_FALSE(d.timebase_valid);
  EXPECT_EQ(d.stamp_ns, 5000000000ULL);

  agt_timebase::SharedTimebaseV1 record;
  record.sequence = 1;
  record.host_update_steady_ns = 1;
  record.lidar_stamp_ns = std::numeric_limits<uint64_t>::max();
  d = agt_hik_camera_driver::choose_camera_stamp(
    record, 0, 2, 250000000ULL, 5000000000ULL);
  EXPECT_FALSE(d.timebase_valid);
  EXPECT_EQ(d.stamp_ns, 5000000000ULL);
}
