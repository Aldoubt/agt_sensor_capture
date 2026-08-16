#include "agt_timebase/shared_timebase.hpp"

#include <utility>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace agt_timebase
{
namespace
{
constexpr std::size_t kRecordSize = sizeof(SharedTimebaseV1);
void * map_record(int fd, int protection) noexcept
{
  void * mapping = ::mmap(nullptr, kRecordSize, protection, MAP_SHARED, fd, 0);
  return mapping == MAP_FAILED ? nullptr : mapping;
}
}  // namespace

SharedTimebaseWriter::SharedTimebaseWriter(std::string path)
: path_(std::move(path))
{
  fd_ = ::open(path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0660);
  if (fd_ < 0) return;
  if (::ftruncate(fd_, static_cast<off_t>(kRecordSize)) != 0) {close_mapping(); return;}
  record_ = static_cast<SharedTimebaseV1 *>(map_record(fd_, PROT_READ | PROT_WRITE));
  if (record_ == nullptr) {close_mapping(); return;}
  if (::flock(fd_, LOCK_EX) == 0) {
    if (record_->magic != kMagic || record_->version != kVersion) {
      *record_ = SharedTimebaseV1{};
      ::msync(record_, kRecordSize, MS_SYNC);
    }
    ::flock(fd_, LOCK_UN);
  } else {
    close_mapping();
  }
}

SharedTimebaseWriter::~SharedTimebaseWriter() {close_mapping();}
SharedTimebaseWriter::SharedTimebaseWriter(SharedTimebaseWriter && other) noexcept
: path_(std::move(other.path_)), fd_(std::exchange(other.fd_, -1)), record_(std::exchange(other.record_, nullptr)) {}
SharedTimebaseWriter & SharedTimebaseWriter::operator=(SharedTimebaseWriter && other) noexcept
{
  if (this != &other) {close_mapping(); path_ = std::move(other.path_); fd_ = std::exchange(other.fd_, -1); record_ = std::exchange(other.record_, nullptr);} return *this;
}
bool SharedTimebaseWriter::valid() const noexcept {return fd_ >= 0 && record_ != nullptr;}
bool SharedTimebaseWriter::write(uint64_t lidar_stamp_ns, uint64_t host_update_steady_ns) noexcept
{
  if (!valid() || lidar_stamp_ns == 0 || ::flock(fd_, LOCK_EX) != 0) return false;
  const uint64_t next_sequence = record_->sequence + 1U;
  record_->magic = kMagic;
  record_->version = kVersion;
  record_->lidar_stamp_ns = lidar_stamp_ns;
  record_->host_update_steady_ns = host_update_steady_ns;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  record_->sequence = next_sequence;
  const bool ok = ::msync(record_, kRecordSize, MS_ASYNC) == 0;
  ::flock(fd_, LOCK_UN);
  return ok;
}
void SharedTimebaseWriter::close_mapping() noexcept
{
  if (record_ != nullptr) {::munmap(record_, kRecordSize); record_ = nullptr;}
  if (fd_ >= 0) {::close(fd_); fd_ = -1;}
}

SharedTimebaseReader::SharedTimebaseReader(std::string path)
: path_(std::move(path))
{
  fd_ = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd_ < 0) return;
  struct stat st {};
  if (::fstat(fd_, &st) != 0 || st.st_size < static_cast<off_t>(kRecordSize)) {close_mapping(); return;}
  record_ = static_cast<const SharedTimebaseV1 *>(map_record(fd_, PROT_READ));
  if (record_ == nullptr) close_mapping();
}
SharedTimebaseReader::~SharedTimebaseReader() {close_mapping();}
SharedTimebaseReader::SharedTimebaseReader(SharedTimebaseReader && other) noexcept
: path_(std::move(other.path_)), fd_(std::exchange(other.fd_, -1)), record_(std::exchange(other.record_, nullptr)) {}
SharedTimebaseReader & SharedTimebaseReader::operator=(SharedTimebaseReader && other) noexcept
{
  if (this != &other) {close_mapping(); path_ = std::move(other.path_); fd_ = std::exchange(other.fd_, -1); record_ = std::exchange(other.record_, nullptr);} return *this;
}
bool SharedTimebaseReader::valid() const noexcept {return fd_ >= 0 && record_ != nullptr;}
std::optional<SharedTimebaseV1> SharedTimebaseReader::read() const noexcept
{
  if (!valid() || ::flock(fd_, LOCK_SH) != 0) return std::nullopt;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  const SharedTimebaseV1 snapshot = *record_;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  ::flock(fd_, LOCK_UN);
  if (snapshot.magic != kMagic || snapshot.version != kVersion || snapshot.sequence == 0 || snapshot.lidar_stamp_ns == 0) return std::nullopt;
  return snapshot;
}
void SharedTimebaseReader::close_mapping() noexcept
{
  if (record_ != nullptr) {::munmap(const_cast<SharedTimebaseV1 *>(record_), kRecordSize); record_ = nullptr;}
  if (fd_ >= 0) {::close(fd_); fd_ = -1;}
}

}  // namespace agt_timebase
