from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src/agt_capture_monitor/src/monitor_node.cpp"


def test_monitor_is_observer_only_and_reports_sync_claims():
    text = SRC.read_text()
    assert '"/agt/sensors/sync/status"' in text
    assert '"/diagnostics"' in text
    assert 'lidar_camera_sync' in text
    assert 'hardware_trigger_driver_timebase' in text
    assert 'gnss_sync' in text
    assert 'independent_gnss_clock_no_pps_bridge' in text
    forbidden_publishers = (
        'create_publisher<livox_ros_driver2::msg::CustomMsg>',
        'create_publisher<sensor_msgs::msg::Imu>',
        'create_publisher<sensor_msgs::msg::Image>',
        'create_publisher<sensor_msgs::msg::NavSatFix>',
    )
    assert not [token for token in forbidden_publishers if token in text]
