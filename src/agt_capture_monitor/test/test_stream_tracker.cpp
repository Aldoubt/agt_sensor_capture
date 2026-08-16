#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include "agt_capture_monitor/stream_tracker.hpp"

using Clock = std::chrono::steady_clock;

TEST(StreamTracker, EstimatesTenHertzAndCountsRollback)
{
  agt_capture_monitor::StreamTracker tracker(std::chrono::seconds(2));
  const auto t0 = Clock::time_point{};
  for (int i = 0; i <= 20; ++i) {
    tracker.observe(1000000000LL + i * 100000000LL, t0 + std::chrono::milliseconds(i * 100));
  }
  EXPECT_NEAR(tracker.rate_hz(), 10.0, 1e-9);
  EXPECT_EQ(tracker.rollback_count(), 0U);
  tracker.observe(500000000LL, t0 + std::chrono::milliseconds(2100));
  EXPECT_EQ(tracker.rollback_count(), 1U);
}

TEST(StreamTracker, EstimatesFiveAndTwoHundredHertz)
{
  const auto t0 = Clock::time_point{};
  agt_capture_monitor::StreamTracker gnss(std::chrono::seconds(2));
  for (int i = 0; i <= 10; ++i) {
    gnss.observe(i * 200000000LL + 1, t0 + std::chrono::milliseconds(i * 200));
  }
  EXPECT_NEAR(gnss.rate_hz(), 5.0, 1e-9);

  agt_capture_monitor::StreamTracker imu(std::chrono::seconds(1));
  for (int i = 0; i <= 200; ++i) {
    imu.observe(i * 5000000LL + 1, t0 + std::chrono::milliseconds(i * 5));
  }
  EXPECT_NEAR(imu.rate_hz(), 200.0, 1e-9);
}
