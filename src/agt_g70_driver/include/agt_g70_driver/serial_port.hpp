#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

namespace agt_g70_driver
{

class ReadOnlySerialPort
{
public:
  ReadOnlySerialPort(std::string path, int baudrate);
  ~ReadOnlySerialPort();
  ReadOnlySerialPort(const ReadOnlySerialPort &) = delete;
  ReadOnlySerialPort & operator=(const ReadOnlySerialPort &) = delete;

  bool open();
  void close() noexcept;
  bool is_open() const noexcept {return fd_ >= 0;}
  ssize_t read_some(uint8_t * buffer, std::size_t capacity, int timeout_ms);

  const std::string & path() const noexcept {return path_;}
  int baudrate() const noexcept {return baudrate_;}
  const std::string & last_error() const noexcept {return last_error_;}

private:
  std::string path_;
  int baudrate_{0};
  int fd_{-1};
  std::string last_error_;
};

}  // namespace agt_g70_driver
