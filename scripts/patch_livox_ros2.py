#!/usr/bin/env python3
from pathlib import Path
import argparse
import shutil
import subprocess

PINNED_COMMIT = "6b9356cadf77084619ba406e6a0eb41163b08039"


def _replace(text: str, before: str, after: str, label: str) -> str:
    if after in text:
        return text
    count = text.count(before)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one upstream anchor, found {count}")
    return text.replace(before, after, 1)


def patch_repo(repo: Path) -> None:
    repo = Path(repo)
    files = {
        "cmake": repo / "CMakeLists.txt",
        "package": repo / "package_ROS2.xml",
        "cpp": repo / "src" / "lddc.cpp",
        "header": repo / "src" / "lddc.h",
    }
    for label, path in files.items():
        if not path.is_file():
            raise RuntimeError(f"{label}: required file not found: {path}")

    cmake = files["cmake"].read_text()
    package = files["package"].read_text()
    cpp = files["cpp"].read_text()
    header = files["header"].read_text()

    cmake = _replace(
        cmake,
        '''  # Default to C++14\n  if(NOT CMAKE_CXX_STANDARD)\n    set(CMAKE_CXX_STANDARD 14)\n  endif()\n\n  list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")''',
        '''  # AGT timebase uses C++17 library interfaces.\n  if(NOT CMAKE_CXX_STANDARD)\n    set(CMAKE_CXX_STANDARD 17)\n  endif()\n\n  # This AGT pin targets ROS 2 Humble unless a distro is supplied explicitly.\n  if(NOT DISTRO_ROS)\n    set(DISTRO_ROS "humble")\n  endif()\n\n  list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")''',
        "CMakeLists.txt C++17/Humble anchor",
    )

    package = _replace(
        package,
        "  <depend>libpcl-all-dev</depend>\n",
        "  <depend>libpcl-all-dev</depend>\n  <depend>agt_timebase</depend>\n",
        "package_ROS2.xml agt_timebase dependency anchor",
    )

    cpp = _replace(
        cpp,
        '#include "comm/comm.h"\n\n#include <inttypes.h>',
        '#include "comm/comm.h"\n\n#include <chrono>\n\n#include <inttypes.h>',
        "lddc.cpp include anchor",
    )

    cpp = _replace(
        cpp,
        '''      frame_id_(frame_id) {\n  publish_period_ns_ = kNsPerSecond / publish_frq_;\n  lds_ = nullptr;\n#if 0\n  bag_ = nullptr;\n#endif\n}\n#endif''',
        '''      frame_id_(frame_id) {\n  publish_period_ns_ = kNsPerSecond / publish_frq_;\n  lds_ = nullptr;\n  shared_timebase_writer_ = std::make_unique<agt_timebase::SharedTimebaseWriter>();\n#if 0\n  bag_ = nullptr;\n#endif\n}\n#endif''',
        "lddc.cpp ROS2 constructor anchor",
    )

    cpp = _replace(
        cpp,
        '''    InitCustomMsg(livox_msg, pkg, index);\n    FillPointsToCustomMsg(livox_msg, pkg);\n    PublishCustomPointData(livox_msg, index);''',
        '''    InitCustomMsg(livox_msg, pkg, index);\n    FillPointsToCustomMsg(livox_msg, pkg);\n    if (shared_timebase_writer_) {\n      const auto host_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(\n          std::chrono::steady_clock::now().time_since_epoch()).count();\n      shared_timebase_writer_->write(\n          static_cast<uint64_t>(pkg.base_time),\n          static_cast<uint64_t>(host_steady_ns));\n    }\n    PublishCustomPointData(livox_msg, index);''',
        "lddc.cpp CustomMsg publish anchor",
    )

    header = _replace(
        header,
        '#include "driver_node.h"\n#include "lds.h"\n\nnamespace livox_ros {',
        '#include "driver_node.h"\n#include "lds.h"\n\n#ifdef BUILDING_ROS2\n#include <memory>\n#include "agt_timebase/shared_timebase.hpp"\n#endif\n\nnamespace livox_ros {',
        "lddc.h include anchor",
    )

    header = _replace(
        header,
        '''#elif defined BUILDING_ROS2\n  PublisherPtr private_pub_[kMaxSourceLidar];\n  PublisherPtr global_pub_;\n  PublisherPtr private_imu_pub_[kMaxSourceLidar];\n  PublisherPtr global_imu_pub_;\n#endif''',
        '''#elif defined BUILDING_ROS2\n  PublisherPtr private_pub_[kMaxSourceLidar];\n  PublisherPtr global_pub_;\n  PublisherPtr private_imu_pub_[kMaxSourceLidar];\n  PublisherPtr global_imu_pub_;\n  std::unique_ptr<agt_timebase::SharedTimebaseWriter> shared_timebase_writer_;\n#endif''',
        "lddc.h writer member anchor",
    )

    checks = {
        "CMakeLists.txt C++17": (cmake, "set(CMAKE_CXX_STANDARD 17)", 1),
        "package agt_timebase": (package, "<depend>agt_timebase</depend>", 1),
        "lddc.cpp write": (cpp, "shared_timebase_writer_->write(", 1),
        "lddc.cpp exact sensor-time write": (
            cpp,
            "shared_timebase_writer_->write(\n          static_cast<uint64_t>(pkg.base_time),",
            1,
        ),
        "lddc.h writer": (header, "shared_timebase_writer_", 1),
    }
    for label, (text, marker, expected) in checks.items():
        actual = text.count(marker)
        if actual != expected:
            raise RuntimeError(f"{label}: expected marker count {expected}, found {actual}")

    files["cmake"].write_text(cmake)
    files["package"].write_text(package)
    files["cpp"].write_text(cpp)
    files["header"].write_text(header)
    shutil.copyfile(files["package"], repo / "package.xml")


def _git_head(repo: Path) -> str | None:
    probe = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "--is-inside-work-tree"],
        check=False,
        text=True,
        capture_output=True,
    )
    if probe.returncode != 0 or probe.stdout.strip() != "true":
        return None
    result = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", type=Path)
    args = parser.parse_args()
    head = _git_head(args.repo)
    if head is not None and head != PINNED_COMMIT:
        raise SystemExit(
            f"livox_ros_driver2 HEAD is {head}, expected pinned {PINNED_COMMIT}"
        )
    patch_repo(args.repo)
    print(f"Prepared livox_ros_driver2 for ROS 2 Humble: {args.repo / 'package.xml'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
