import importlib.util
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools/inspect_bag.py"


def load_module():
    spec = importlib.util.spec_from_file_location("inspect_bag", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_analyze_info_accepts_expected_rates_and_rejects_missing_topic():
    mod = load_module()
    info = {
        "rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 10_000_000_000},
            "topics_with_message_count": [
                {"topic_metadata": {"name": "/agt/sensors/lidar/custom"}, "message_count": 100},
                {"topic_metadata": {"name": "/agt/sensors/imu/data"}, "message_count": 2000},
                {"topic_metadata": {"name": "/agt/sensors/camera/front/image_raw"}, "message_count": 100},
                {"topic_metadata": {"name": "/agt/sensors/camera/front/camera_info"}, "message_count": 100},
                {"topic_metadata": {"name": "/agt/sensors/camera/front/status"}, "message_count": 10},
                {"topic_metadata": {"name": "/agt/sensors/gnss/fix"}, "message_count": 50},
                {"topic_metadata": {"name": "/agt/sensors/gnss/velocity"}, "message_count": 50},
                {"topic_metadata": {"name": "/agt/sensors/gnss/time_reference"}, "message_count": 50},
                {"topic_metadata": {"name": "/agt/sensors/gnss/status"}, "message_count": 50},
                {"topic_metadata": {"name": "/agt/sensors/sync/status"}, "message_count": 10},
                {"topic_metadata": {"name": "/diagnostics"}, "message_count": 20},
                {"topic_metadata": {"name": "/tf"}, "message_count": 1},
                {"topic_metadata": {"name": "/tf_static"}, "message_count": 1},
            ],
        }
    }
    report = mod.analyze_info(info)
    assert report["ok"] is True
    assert report["rates_hz"]["/agt/sensors/imu/data"] == 200.0

    info["rosbag2_bagfile_information"]["topics_with_message_count"] = [
        row for row in info["rosbag2_bagfile_information"]["topics_with_message_count"]
        if row["topic_metadata"]["name"] != "/agt/sensors/gnss/status"
    ]
    report = mod.analyze_info(info)
    assert report["ok"] is False
    assert "/agt/sensors/gnss/status" in report["missing_topics"]


def test_main_reads_rosbag_metadata_yaml_without_non_humble_yaml_cli(tmp_path, capsys):
    mod = load_module()
    bag = tmp_path / "rosbag2"
    bag.mkdir()
    info = {
        "rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 10_000_000_000},
            "topics_with_message_count": [
                {"topic_metadata": {"name": topic}, "message_count": count}
                for topic, count in {
                    "/agt/sensors/lidar/custom": 100,
                    "/agt/sensors/imu/data": 2000,
                    "/agt/sensors/camera/front/image_raw": 100,
                    "/agt/sensors/camera/front/camera_info": 100,
                    "/agt/sensors/camera/front/status": 10,
                    "/agt/sensors/gnss/fix": 50,
                    "/agt/sensors/gnss/velocity": 50,
                    "/agt/sensors/gnss/time_reference": 50,
                    "/agt/sensors/gnss/status": 50,
                    "/agt/sensors/sync/status": 10,
                    "/diagnostics": 20,
                }.items()
            ],
        }
    }
    (bag / "metadata.yaml").write_text(yaml.safe_dump(info))
    assert mod.main([str(bag)]) == 0
    assert '"ok": true' in capsys.readouterr().out
