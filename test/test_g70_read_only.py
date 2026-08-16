from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "agt_g70_driver"


def test_runtime_g70_driver_is_read_only():
    serial_header = (PKG / "include/agt_g70_driver/serial_port.hpp").read_text()
    node_source = (PKG / "src/g70_node.cpp").read_text()
    assert re.search(r"\bwrite\s*\(", serial_header) is None
    forbidden = ("CFG-", "configRate", "setDynamicModel", "Survey-In", "TMODE")
    assert not [token for token in forbidden if token in node_source]


def test_g70_default_config_uses_alias_and_5hz_receiver_contract():
    text = (PKG / "config/g70.yaml").read_text()
    assert 'port: "/dev/wheeltec_gnss"' in text
    assert "baudrate: 9600" in text
    assert "expected_rate_hz: 5.0" in text
