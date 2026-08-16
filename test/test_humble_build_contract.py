from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LIVOX_PIN = "6b9356cadf77084619ba406e6a0eb41163b08039"


def test_humble_sources_do_not_use_rclcpp_time_to_msg():
    sources = list((ROOT / "src").rglob("*.cpp")) + list((ROOT / "src").rglob("*.hpp"))
    offenders = [str(source.relative_to(ROOT)) for source in sources if ".to_msg()" in source.read_text()]
    assert not offenders, offenders


def test_livox_pin_matches_mid360_compatible_baseline():
    repos = (ROOT / "dependencies.repos").read_text()
    patcher = (ROOT / "scripts/patch_livox_ros2.py").read_text()
    assert f"version: {LIVOX_PIN}" in repos
    assert f'PINNED_COMMIT = "{LIVOX_PIN}"' in patcher
