#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import yaml

REQUIRED_TOPICS = {
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
}

RATE_LIMITS_HZ = {
    "/agt/sensors/lidar/custom": (9.5, 10.5),
    "/agt/sensors/imu/data": (180.0, 220.0),
    "/agt/sensors/camera/front/image_raw": (9.5, 10.5),
    "/agt/sensors/gnss/status": (4.5, 5.5),
}


def _unwrap(info):
    return info.get("rosbag2_bagfile_information", info)


def analyze_info(info):
    bag = _unwrap(info)
    duration_ns = int((bag.get("duration") or {}).get("nanoseconds", 0))
    duration_s = duration_ns / 1e9 if duration_ns > 0 else 0.0
    rows = bag.get("topics_with_message_count") or []
    counts = {}
    for row in rows:
        metadata = row.get("topic_metadata") or {}
        name = metadata.get("name")
        if name:
            counts[name] = int(row.get("message_count", 0))

    missing = sorted(REQUIRED_TOPICS - set(counts))
    rates = {}
    rate_failures = []
    if duration_s <= 0:
        rate_failures.append("bag duration is zero or unavailable")
    else:
        for topic, (minimum, maximum) in RATE_LIMITS_HZ.items():
            if topic not in counts:
                continue
            rate = counts[topic] / duration_s
            rates[topic] = rate
            if rate < minimum or rate > maximum:
                rate_failures.append(
                    f"{topic}: {rate:.3f} Hz outside [{minimum:.3f}, {maximum:.3f}]"
                )

    return {
        "ok": not missing and not rate_failures,
        "duration_s": duration_s,
        "missing_topics": missing,
        "rate_failures": rate_failures,
        "rates_hz": rates,
        "message_counts": counts,
    }


def _bag_path(path: Path) -> Path:
    if (path / "rosbag2").is_dir():
        return path / "rosbag2"
    return path


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    args = parser.parse_args(argv)
    bag = _bag_path(args.bag.expanduser())
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        print(json.dumps({"ok": False, "error": f"missing rosbag2 metadata: {metadata_path}"}, indent=2))
        return 2
    report = analyze_info(yaml.safe_load(metadata_path.read_text()) or {})
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
