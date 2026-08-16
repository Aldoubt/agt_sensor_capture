#include "agt_hik_camera_driver/timestamp_policy.hpp"

#include <limits>

namespace agt_hik_camera_driver
{

CameraStampDecision choose_camera_stamp(
  const std::optional<agt_timebase::SharedTimebaseV1> & record,
  uint64_t previous_sequence,
  uint64_t host_steady_now_ns,
  uint64_t stale_threshold_ns,
  uint64_t fallback_ros_now_ns) noexcept
{
  CameraStampDecision decision;
  if (!record.has_value() || record->magic != agt_timebase::kMagic ||
      record->version != agt_timebase::kVersion || record->sequence == 0 ||
      record->lidar_stamp_ns == 0 ||
      record->lidar_stamp_ns > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) ||
      record->host_update_steady_ns == 0)
  {
    decision.stamp_ns = fallback_ros_now_ns;
    return decision;
  }

  decision.stamp_ns = record->lidar_stamp_ns;
  decision.sequence = record->sequence;
  decision.timebase_valid = true;
  decision.repeated = previous_sequence != 0 && record->sequence == previous_sequence;
  decision.stale = host_steady_now_ns < record->host_update_steady_ns ||
    host_steady_now_ns - record->host_update_steady_ns > stale_threshold_ns;
  return decision;
}

}  // namespace agt_hik_camera_driver
