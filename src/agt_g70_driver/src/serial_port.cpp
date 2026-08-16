#include "agt_g70_driver/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace agt_g70_driver
{
namespace
{

bool baud_to_speed(int baudrate, speed_t & speed)
{
  switch (baudrate) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 38400: speed = B38400; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
#ifdef B230400
    case 230400: speed = B230400; return true;
#endif
    default: return false;
  }
}

std::string system_error(const char * prefix)
{
  return std::string(prefix) + ": " + std::strerror(errno);
}

}  // namespace

ReadOnlySerialPort::ReadOnlySerialPort(std::string path, int baudrate)
: path_(std::move(path)), baudrate_(baudrate)
{
}

ReadOnlySerialPort::~ReadOnlySerialPort()
{
  close();
}

bool ReadOnlySerialPort::open()
{
  close();
  last_error_.clear();

  speed_t speed{};
  if (!baud_to_speed(baudrate_, speed)) {
    last_error_ = "unsupported baudrate: " + std::to_string(baudrate_);
    return false;
  }

  fd_ = ::open(path_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    last_error_ = system_error("open failed");
    return false;
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    last_error_ = system_error("tcgetattr failed");
    close();
    return false;
  }
  ::cfmakeraw(&tty);
  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  tty.c_cflag = static_cast<tcflag_t>((tty.c_cflag & ~CSIZE) | CS8);
  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0 ||
      ::tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    last_error_ = system_error("serial configuration failed");
    close();
    return false;
  }
  ::tcflush(fd_, TCIFLUSH);
  return true;
}

void ReadOnlySerialPort::close() noexcept
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t ReadOnlySerialPort::read_some(
  uint8_t * buffer, std::size_t capacity, int timeout_ms)
{
  if (!is_open() || buffer == nullptr || capacity == 0) {
    last_error_ = "read requested on invalid serial port or buffer";
    return -1;
  }

  pollfd descriptor{};
  descriptor.fd = fd_;
  descriptor.events = POLLIN;
  const int poll_result = ::poll(&descriptor, 1, timeout_ms);
  if (poll_result == 0) return 0;
  if (poll_result < 0) {
    if (errno == EINTR) return 0;
    last_error_ = system_error("poll failed");
    return -1;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    last_error_ = "serial poll reported device error/hangup";
    return -1;
  }

  const ssize_t result = ::read(fd_, buffer, capacity);
  if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
  if (result < 0) last_error_ = system_error("read failed");
  return result;
}

}  // namespace agt_g70_driver
