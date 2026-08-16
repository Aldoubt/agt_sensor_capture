#!/usr/bin/env python3
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from livox_ros_driver2.msg import CustomMsg
from agt_capture_msgs.msg import GnssStatus


def header_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def lidar_ns(msg):
    return int(msg.timebase) if int(msg.timebase) > 0 else header_ns(msg)


class StampStats:
    def __init__(self):
        self.count = 0
        self.duplicates = 0
        self.rollbacks = 0
        self.min_delta_ns = None
        self.max_delta_ns = None
        self.last = None

    def observe(self, stamp):
        if self.last is not None:
            delta = stamp - self.last
            if delta == 0:
                self.duplicates += 1
            elif delta < 0:
                self.rollbacks += 1
            else:
                self.min_delta_ns = delta if self.min_delta_ns is None else min(self.min_delta_ns, delta)
                self.max_delta_ns = delta if self.max_delta_ns is None else max(self.max_delta_ns, delta)
        self.last = stamp
        self.count += 1

    def report(self):
        return {
            "count": self.count,
            "duplicates": self.duplicates,
            "rollbacks": self.rollbacks,
            "min_delta_ns": self.min_delta_ns,
            "max_delta_ns": self.max_delta_ns,
        }


class TimestampChecker(Node):
    def __init__(self):
        super().__init__("agt_check_timestamps")
        self.stats = {name: StampStats() for name in ("lidar", "imu", "camera", "gnss")}
        self.subscriptions = [
            self.create_subscription(CustomMsg, "/agt/sensors/lidar/custom", lambda m: self.stats["lidar"].observe(lidar_ns(m)), qos_profile_sensor_data),
            self.create_subscription(Imu, "/agt/sensors/imu/data", lambda m: self.stats["imu"].observe(header_ns(m)), qos_profile_sensor_data),
            self.create_subscription(Image, "/agt/sensors/camera/front/image_raw", lambda m: self.stats["camera"].observe(header_ns(m)), qos_profile_sensor_data),
            self.create_subscription(GnssStatus, "/agt/sensors/gnss/status", lambda m: self.stats["gnss"].observe(header_ns(m)), 20),
        ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = TimestampChecker()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        report = {name: stats.report() for name, stats in node.stats.items()}
        print(json.dumps(report, indent=2, sort_keys=True))
        ok = all(stats.rollbacks == 0 and stats.count > 0 for stats in node.stats.values())
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
