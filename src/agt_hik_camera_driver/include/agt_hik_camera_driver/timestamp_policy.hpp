#pragma once

#include <cstdint>
#include <optional>

#include "agt_timebase/shared_timebase.hpp"

namespace agt_hik_camera_driver
{

struct CameraStampDecision
{
  uint64_t stamp_ns{0};
  uint64_t sequence{0};
  bool timebase_valid{false};
  bool stale{false};
  bool repeated{false};
};

CameraStampDecision choose_camera_stamp(
  const std::optional<agt_timebase::SharedTimebaseV1> & record,
  uint64_t previous_sequence,
  uint64_t host_steady_now_ns,
  uint64_t stale_threshold_ns,
  uint64_t fallback_ros_now_ns) noexcept;

}  // namespace agt_hik_camera_driver
