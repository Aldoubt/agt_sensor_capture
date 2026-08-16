from datetime import datetime, timezone
from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration


_VALID_SESSION = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def _start_recording(context):
    session_name = LaunchConfiguration("session_name").perform(context)
    output_root = Path(LaunchConfiguration("output_root").perform(context)).expanduser()
    if not _VALID_SESSION.fullmatch(session_name):
        raise RuntimeError(
            "session_name must start with an alphanumeric character and contain only A-Z, a-z, 0-9, _, . or -"
        )

    share = Path(get_package_share_directory("agt_capture_bringup"))
    topics_file = share / "config" / "record_topics.yaml"
    topics = yaml.safe_load(topics_file.read_text())["topics"]
    session_dir = output_root / session_name
    if session_dir.exists():
        raise RuntimeError(f"Capture session already exists: {session_dir}")
    session_dir.mkdir(parents=True)

    metadata = f"""schema_version: 1
session_name: {session_name}
created_utc: {datetime.now(timezone.utc).isoformat()}
sync:
  lidar_camera: hardware_trigger_driver_timebase
  gnss: independent_gnss_clock_no_pps_bridge
expected_rates_hz:
  lidar: 10.0
  imu: 200.0
  camera: 10.0
  gnss: 5.0
record_topics_file: {topics_file}
"""
    (session_dir / "session.yaml").write_text(metadata)

    return [
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "--storage", "sqlite3",
                "--output", str(session_dir / "rosbag2"),
                *topics,
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("session_name"),
            DeclareLaunchArgument("output_root", default_value="datasets"),
            OpaqueFunction(function=_start_recording),
        ]
    )
