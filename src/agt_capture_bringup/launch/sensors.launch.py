from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_paths(context):
    checks = (
        ("mid360_config", LaunchConfiguration("mid360_config").perform(context)),
        ("camera_params", LaunchConfiguration("camera_params").perform(context)),
        ("g70_params", LaunchConfiguration("g70_params").perform(context)),
    )
    missing = [f"{name}={path}" for name, path in checks if not Path(path).expanduser().is_file()]
    if missing:
        raise RuntimeError("Missing sensor configuration file(s): " + ", ".join(missing))
    return []


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("agt_capture_bringup"))
    camera_share = Path(get_package_share_directory("agt_hik_camera_driver"))
    g70_share = Path(get_package_share_directory("agt_g70_driver"))

    mid360_config = LaunchConfiguration("mid360_config")
    camera_params = LaunchConfiguration("camera_params")
    g70_params = LaunchConfiguration("g70_params")
    g70_port = LaunchConfiguration("g70_port")
    g70_baud = LaunchConfiguration("g70_baud")
    timebase_path = LaunchConfiguration("timebase_path")
    camera_start_delay_s = LaunchConfiguration("camera_start_delay_s")

    livox = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="agt_sensor_mid360_driver",
        output="screen",
        parameters=[
            {
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "user_config_path": mid360_config,
            }
        ],
        remappings=[
            ("/livox/lidar", "/agt/sensors/lidar/custom"),
            ("/livox/imu", "/agt/sensors/imu/data"),
        ],
    )

    camera = Node(
        package="agt_hik_camera_driver",
        executable="camera_node",
        name="agt_hik_camera_driver",
        output="screen",
        parameters=[camera_params, {"timebase_path": timebase_path}],
    )

    g70 = Node(
        package="agt_g70_driver",
        executable="g70_node",
        name="agt_g70_driver",
        output="screen",
        parameters=[g70_params, {"port": g70_port, "baudrate": g70_baud}],
    )

    monitor = Node(
        package="agt_capture_monitor",
        executable="monitor_node",
        name="agt_capture_monitor",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mid360_config",
                default_value=str(bringup_share / "config" / "mid360_network.json"),
            ),
            DeclareLaunchArgument(
                "camera_params",
                default_value=str(camera_share / "config" / "camera.yaml"),
            ),
            DeclareLaunchArgument(
                "g70_params",
                default_value=str(g70_share / "config" / "g70.yaml"),
            ),
            DeclareLaunchArgument("g70_port", default_value="/dev/wheeltec_gnss"),
            DeclareLaunchArgument("g70_baud", default_value="9600"),
            DeclareLaunchArgument("timebase_path", default_value="/dev/shm/agt_livox_timebase"),
            DeclareLaunchArgument("camera_start_delay_s", default_value="2.0"),
            OpaqueFunction(function=_validate_paths),
            livox,
            g70,
            monitor,
            TimerAction(period=camera_start_delay_s, actions=[camera]),
        ]
    )
