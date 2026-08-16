#include <cstdint>
#include <unistd.h>
#include <pty.h>
#include <gtest/gtest.h>
#include "agt_g70_driver/serial_port.hpp"

TEST(ReadOnlySerialPort, ReadsFromPseudoTerminal)
{
  int master = -1;
  int slave = -1;
  char name[128]{};
  ASSERT_EQ(::openpty(&master, &slave, name, nullptr, nullptr), 0);
  ::close(slave);
  agt_g70_driver::ReadOnlySerialPort port(name, 9600);
  ASSERT_TRUE(port.open()) << port.last_error();
  const uint8_t tx[]{0xB5, 0x62, 0x01};
  ASSERT_EQ(::write(master, tx, sizeof(tx)), static_cast<ssize_t>(sizeof(tx)));
  uint8_t rx[8]{};
  ASSERT_EQ(port.read_some(rx, sizeof(rx), 100), 3);
  EXPECT_EQ(rx[0], 0xB5);
  EXPECT_EQ(rx[2], 0x01);
  ::close(master);
}

TEST(ReadOnlySerialPort, RejectsUnsupportedBaudrate)
{
  agt_g70_driver::ReadOnlySerialPort port("/dev/null", 12345);
  EXPECT_FALSE(port.open());
  EXPECT_NE(port.last_error().find("unsupported baudrate"), std::string::npos);
}
