#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace agt_timebase
{

inline constexpr uint32_t kMagic = 0x41475431U;  // "AGT1"
inline constexpr uint32_t kVersion = 1U;
inline constexpr const char * kDefaultPath = "/dev/shm/agt_livox_timebase";

struct SharedTimebaseV1
{
  uint32_t magic{kMagic};
  uint32_t version{kVersion};
  uint64_t sequence{0};
  uint64_t lidar_stamp_ns{0};
  uint64_t host_update_steady_ns{0};
};

static_assert(sizeof(SharedTimebaseV1) == 32, "SharedTimebaseV1 layout changed");

class SharedTimebaseWriter
{
public:
  explicit SharedTimebaseWriter(std::string path = kDefaultPath);
  ~SharedTimebaseWriter();
  SharedTimebaseWriter(const SharedTimebaseWriter &) = delete;
  SharedTimebaseWriter & operator=(const SharedTimebaseWriter &) = delete;
  SharedTimebaseWriter(SharedTimebaseWriter && other) noexcept;
  SharedTimebaseWriter & operator=(SharedTimebaseWriter && other) noexcept;
  bool valid() const noexcept;
  bool write(uint64_t lidar_stamp_ns, uint64_t host_update_steady_ns) noexcept;
  const std::string & path() const noexcept {return path_;}
private:
  void close_mapping() noexcept;
  std::string path_;
  int fd_{-1};
  SharedTimebaseV1 * record_{nullptr};
};

class SharedTimebaseReader
{
public:
  explicit SharedTimebaseReader(std::string path = kDefaultPath);
  ~SharedTimebaseReader();
  SharedTimebaseReader(const SharedTimebaseReader &) = delete;
  SharedTimebaseReader & operator=(const SharedTimebaseReader &) = delete;
  SharedTimebaseReader(SharedTimebaseReader && other) noexcept;
  SharedTimebaseReader & operator=(SharedTimebaseReader && other) noexcept;
  bool valid() const noexcept;
  std::optional<SharedTimebaseV1> read() const noexcept;
  const std::string & path() const noexcept {return path_;}
private:
  void close_mapping() noexcept;
  std::string path_;
  int fd_{-1};
  const SharedTimebaseV1 * record_{nullptr};
};

}  // namespace agt_timebase
