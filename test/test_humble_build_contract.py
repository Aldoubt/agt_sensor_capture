from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LIVOX_PIN = "6b9356cadf77084619ba406e6a0eb41163b08039"


def test_humble_sources_do_not_use_rclcpp_time_to_msg():
    sources = [
        ROOT / "src/agt_hik_camera_driver/src/camera_node.cpp",
        ROOT / "src/agt_g70_driver/src/ros_conversion.cpp",
        ROOT / "src/agt_capture_monitor/src/monitor_node.cpp",
    ]
    for source in sources:
        assert ".to_msg()" not in source.read_text(), source


def test_livox_pin_matches_mid360_compatible_baseline():
    repos = (ROOT / "dependencies.repos").read_text()
    patcher = (ROOT / "scripts/patch_livox_ros2.py").read_text()
    assert f"version: {LIVOX_PIN}" in repos
    assert f'PINNED_COMMIT = "{LIVOX_PIN}"' in patcher
