from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PATCHER = ROOT / "scripts" / "patch_livox_ros2.py"

CMAKE = '''else(ROS_EDITION STREQUAL "ROS2")
  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")
'''

PACKAGE = '''<package format="3">
  <member_of_group>rosidl_interface_packages</member_of_group>
  <depend>pcl_conversions</depend>
  <depend>rcl_interfaces</depend>
  <depend>libpcl-all-dev</depend>

  <exec_depend>rosbag2</exec_depend>
</package>
'''

CPP = '''#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"

#include <inttypes.h>

#elif defined BUILDING_ROS2
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
           double frq, std::string &frame_id)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
#if 0
  bag_ = nullptr;
#endif
}
#endif

void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      continue;
    }
    CustomMsg livox_msg;
    InitCustomMsg(livox_msg, pkg, index);
    FillPointsToCustomMsg(livox_msg, pkg);
    PublishCustomPointData(livox_msg, index);
  }
}
'''

HEADER = '''#include "include/livox_ros_driver2.h"

#include "driver_node.h"
#include "lds.h"

namespace livox_ros {

#ifdef BUILDING_ROS1
  bool enable_lidar_bag_;
#elif defined BUILDING_ROS2
  PublisherPtr private_pub_[kMaxSourceLidar];
  PublisherPtr global_pub_;
  PublisherPtr private_imu_pub_[kMaxSourceLidar];
  PublisherPtr global_imu_pub_;
#endif

  livox_ros::DriverNode *cur_node_;
};
'''


def make_fixture(tmp_path: Path) -> Path:
    repo = tmp_path / "livox_ros_driver2"
    (repo / "src").mkdir(parents=True)
    (repo / "CMakeLists.txt").write_text(CMAKE)
    (repo / "package_ROS2.xml").write_text(PACKAGE)
    (repo / "src/lddc.cpp").write_text(CPP)
    (repo / "src/lddc.h").write_text(HEADER)
    return repo


def run_patcher(repo: Path):
    return subprocess.run(
        [sys.executable, str(PATCHER), str(repo)],
        text=True,
        capture_output=True,
        check=False,
    )


def test_livox_patcher_is_idempotent_and_creates_ros2_package_xml(tmp_path):
    repo = make_fixture(tmp_path)
    first = run_patcher(repo)
    second = run_patcher(repo)
    assert first.returncode == 0, first.stderr
    assert second.returncode == 0, second.stderr
    assert (repo / "package.xml").read_text() == (repo / "package_ROS2.xml").read_text()
    assert "set(CMAKE_CXX_STANDARD 17)" in (repo / "CMakeLists.txt").read_text()
    assert "<depend>agt_timebase</depend>" in (repo / "package.xml").read_text()
    assert (repo / "src/lddc.cpp").read_text().count("shared_timebase_writer_->write(") == 1
    assert (repo / "src/lddc.h").read_text().count("shared_timebase_writer_") == 1


def test_livox_patcher_reports_the_specific_missing_anchor(tmp_path):
    repo = make_fixture(tmp_path)
    (repo / "src/lddc.cpp").write_text("unexpected source")
    result = run_patcher(repo)
    assert result.returncode != 0
    assert "lddc.cpp include anchor" in result.stderr
