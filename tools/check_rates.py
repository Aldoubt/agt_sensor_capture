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

STREAMS = {
    "lidar": ("/agt/sensors/lidar/custom", CustomMsg, (9.5, 10.5)),
    "imu": ("/agt/sensors/imu/data", Imu, (180.0, 220.0)),
    "camera": ("/agt/sensors/camera/front/image_raw", Image, (9.5, 10.5)),
    "gnss": ("/agt/sensors/gnss/status", GnssStatus, (4.5, 5.5)),
}


class RateChecker(Node):
    def __init__(self):
        super().__init__("agt_check_rates")
        self.times = {name: [] for name in STREAMS}
        self.subscriptions = []
        for name, (topic, msg_type, _) in STREAMS.items():
            qos = qos_profile_sensor_data if name in {"lidar", "imu", "camera"} else 20
            self.subscriptions.append(
                self.create_subscription(msg_type, topic, self._callback(name), qos)
            )

    def _callback(self, name):
        def callback(_msg):
            self.times[name].append(time.monotonic())
        return callback


def measured_rate(samples):
    if len(samples) < 2:
        return 0.0
    elapsed = samples[-1] - samples[0]
    return (len(samples) - 1) / elapsed if elapsed > 0 else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = RateChecker()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        report = {}
        ok = True
        for name, (_, _, limits) in STREAMS.items():
            rate = measured_rate(node.times[name])
            stream_ok = limits[0] <= rate <= limits[1]
            report[name] = {"rate_hz": rate, "samples": len(node.times[name]), "ok": stream_ok}
            ok = ok and stream_ok
        print(json.dumps(report, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
