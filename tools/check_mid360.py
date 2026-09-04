#!/usr/bin/env python3
"""Validate the simulated MID-360 PointCloud2 stream."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class Mid360Validator(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("mid360_validator")
        self.messages = []
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        self.messages.append((time.monotonic(), message))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check the simulated MID-360 PointCloud2 stream."
    )
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--frame", default="mid360_link")
    parser.add_argument("--messages", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init(args=[])
    node = Mid360Validator(args.topic)
    deadline = time.monotonic() + args.timeout

    try:
        while len(node.messages) < args.messages and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not node.messages:
        print(f"[FAIL] No PointCloud2 messages received on {args.topic}")
        print("       Start gazebo_mid360.launch.py in another terminal first.")
        return 2

    first_time = node.messages[0][0]
    last_time, cloud = node.messages[-1]
    received = len(node.messages)
    point_count = cloud.width * cloud.height
    field_names = {field.name for field in cloud.fields}
    required_fields = {"x", "y", "z"}
    rate = (
        (received - 1) / (last_time - first_time)
        if received > 1 and last_time > first_time
        else 0.0
    )

    checks = {
        "messages received": received >= args.messages,
        "non-empty cloud": point_count > 0 and len(cloud.data) > 0,
        f"frame_id is {args.frame}": cloud.header.frame_id == args.frame,
        "XYZ fields present": required_fields.issubset(field_names),
    }

    print(f"Topic       : {args.topic}")
    print(f"Frame       : {cloud.header.frame_id}")
    print(f"Messages    : {received}")
    print(f"Points/frame: {point_count}")
    print(f"Rate        : {rate:.2f} Hz")
    print(f"Fields      : {', '.join(sorted(field_names))}")

    failed = False
    for label, passed in checks.items():
        print(f"[{'PASS' if passed else 'FAIL'}] {label}")
        failed = failed or not passed

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
