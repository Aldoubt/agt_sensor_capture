#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include "agt_timebase/shared_timebase.hpp"

namespace
{
std::string temp_path(const char * suffix)
{
  return (std::filesystem::temp_directory_path() / (std::string("agt_timebase_") + suffix + ".bin")).string();
}
}

TEST(SharedTimebase, RoundTripPreservesStampAndSequence)
{
  const auto path = temp_path("roundtrip");
  std::filesystem::remove(path);
  agt_timebase::SharedTimebaseWriter writer(path);
  agt_timebase::SharedTimebaseReader reader(path);
  EXPECT_FALSE(reader.read().has_value());
  ASSERT_TRUE(writer.write(123456789ULL, 987654321ULL));
  auto value = reader.read();
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(value->lidar_stamp_ns, 123456789ULL);
  EXPECT_EQ(value->host_update_steady_ns, 987654321ULL);
  EXPECT_EQ(value->sequence, 1ULL);
  ASSERT_TRUE(writer.write(223456789ULL, 887654321ULL));
  value = reader.read();
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(value->sequence, 2ULL);
  std::filesystem::remove(path);
}

TEST(SharedTimebase, RejectsInvalidMagicAndVersion)
{
  const auto path = temp_path("invalid");
  std::filesystem::remove(path);
  {
    agt_timebase::SharedTimebaseWriter writer(path);
    ASSERT_TRUE(writer.write(1ULL, 2ULL));
  }
  std::fstream file(path, std::ios::in | std::ios::out | std::ios::binary);
  uint32_t zero = 0;
  file.write(reinterpret_cast<const char *>(&zero), sizeof(zero));
  file.flush();
  agt_timebase::SharedTimebaseReader reader(path);
  EXPECT_FALSE(reader.read().has_value());
  std::filesystem::remove(path);
}
