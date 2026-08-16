from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_v01_timebase_path_is_frozen_across_writer_and_camera_bringup():
    launch = (ROOT / "src/agt_capture_bringup/launch/sensors.launch.py").read_text()
    camera = (ROOT / "src/agt_hik_camera_driver/config/camera.yaml").read_text()
    header = (ROOT / "src/agt_timebase/include/agt_timebase/shared_timebase.hpp").read_text()
    assert 'DeclareLaunchArgument("timebase_path"' not in launch
    assert '"timebase_path":' not in launch
    assert 'timebase_path: "/dev/shm/agt_livox_timebase"' in camera
    assert 'kDefaultPath = "/dev/shm/agt_livox_timebase"' in header


def test_timestamp_checker_treats_duplicate_sensor_stamps_as_failure():
    text = (ROOT / "tools/check_timestamps.py").read_text()
    assert "stats.duplicates == 0" in text
    assert "stats.rollbacks == 0" in text
