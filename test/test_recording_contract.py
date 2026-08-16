from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "src/agt_capture_bringup/config/record_topics.yaml"
REQUIRED = {
    "/agt/sensors/lidar/custom",
    "/agt/sensors/imu/data",
    "/agt/sensors/camera/front/image_raw",
    "/agt/sensors/camera/front/camera_info",
    "/agt/sensors/camera/front/status",
    "/agt/sensors/gnss/fix",
    "/agt/sensors/gnss/velocity",
    "/agt/sensors/gnss/time_reference",
    "/agt/sensors/gnss/status",
    "/agt/sensors/sync/status",
    "/diagnostics",
    "/tf",
    "/tf_static",
}


def test_recording_contract_uses_only_canonical_sensor_topics():
    data = yaml.safe_load(CFG.read_text())
    topics = set(data["topics"])
    assert REQUIRED <= topics
    assert not [t for t in topics if t.startswith("/livox/") or t.startswith("/left_camera/")]
