#include "agt_capture_monitor/stream_tracker.hpp"

#include <stdexcept>

namespace agt_capture_monitor
{

StreamTracker::StreamTracker(std::chrono::steady_clock::duration window)
: window_(window)
{
  if (window_ <= std::chrono::steady_clock::duration::zero()) {
    throw std::invalid_argument("StreamTracker window must be positive");
  }
}

void StreamTracker::observe(
  int64_t stamp_ns,
  std::chrono::steady_clock::time_point receive_time)
{
  if (have_stamp_ && stamp_ns < last_stamp_ns_) {
    ++rollback_count_;
  }
  last_stamp_ns_ = stamp_ns;
  have_stamp_ = true;
  ++sample_count_;
  receive_times_.push_back(receive_time);
  prune(receive_time);
}

void StreamTracker::prune(std::chrono::steady_clock::time_point newest)
{
  while (receive_times_.size() > 2 && newest - receive_times_.front() > window_) {
    receive_times_.pop_front();
  }
}

double StreamTracker::rate_hz() const noexcept
{
  if (receive_times_.size() < 2) return 0.0;
  const auto elapsed = std::chrono::duration<double>(
    receive_times_.back() - receive_times_.front()).count();
  if (elapsed <= 0.0) return 0.0;
  return static_cast<double>(receive_times_.size() - 1U) / elapsed;
}

}  // namespace agt_capture_monitor
