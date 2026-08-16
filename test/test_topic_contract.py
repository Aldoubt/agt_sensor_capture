from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    "/agt/sensors/lidar/custom",
    "/agt/sensors/imu/data",
    "/agt/sensors/camera/front/image_raw",
    "/agt/sensors/camera/front/camera_info",
    "/agt/sensors/gnss/fix",
    "/agt/sensors/gnss/velocity",
    "/agt/sensors/gnss/time_reference",
    "/agt/sensors/gnss/status",
    "/agt/sensors/sync/status",
}

def test_topic_contract_contains_all_canonical_topics():
    text = (ROOT / "docs" / "TOPIC_CONTRACT.md").read_text()
    missing = sorted(topic for topic in EXPECTED if topic not in text)
    assert not missing, f"missing canonical topics: {missing}"
