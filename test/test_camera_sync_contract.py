from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "agt_hik_camera_driver"


def test_camera_driver_has_no_ros_livox_pairing_path():
    text = (PKG / "src/camera_node.cpp").read_text()
    forbidden = (
        "livox_ros_driver2",
        "create_subscription",
        "matchImage",
        "max_pairing_host_delta",
        "sync_wait_timeout",
        "Dropping camera frame",
    )
    assert not [token for token in forbidden if token in text]


def test_camera_default_contract_is_line0_10hz_rgb8():
    text = (PKG / "config/camera.yaml").read_text()
    assert 'trigger_source: "Line0"' in text
    assert 'trigger_activation: "FallingEdge"' in text
    assert 'pixel_format: "RGB8Packed"' in text
    assert "exposure_time_us: 5000.0" in text
    assert "expected_rate_hz: 10.0" in text
