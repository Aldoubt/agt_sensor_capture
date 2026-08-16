#pragma once

#include <chrono>
#include <cstdint>
#include <deque>

namespace agt_capture_monitor
{

class StreamTracker
{
public:
  explicit StreamTracker(std::chrono::steady_clock::duration window);

  void observe(
    int64_t stamp_ns,
    std::chrono::steady_clock::time_point receive_time);

  double rate_hz() const noexcept;
  uint64_t rollback_count() const noexcept {return rollback_count_;}
  uint64_t sample_count() const noexcept {return sample_count_;}
  int64_t last_stamp_ns() const noexcept {return last_stamp_ns_;}

private:
  void prune(std::chrono::steady_clock::time_point newest);

  std::chrono::steady_clock::duration window_;
  std::deque<std::chrono::steady_clock::time_point> receive_times_;
  int64_t last_stamp_ns_{0};
  bool have_stamp_{false};
  uint64_t rollback_count_{0};
  uint64_t sample_count_{0};
};

}  // namespace agt_capture_monitor
