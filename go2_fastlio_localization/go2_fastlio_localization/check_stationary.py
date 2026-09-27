#!/usr/bin/env python3
"""Report FAST-LIO drift while a stationary rosbag is replayed."""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DriftCheck(Node):
    def __init__(self):
        super().__init__("fastlio_stationary_drift_check")
        self.poses = []
        self.stamps = []
        self.create_subscription(Odometry, "/Odometry", self.on_odom, 100)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.poses.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def report(self):
        if not self.poses:
            print("No /Odometry messages received.", flush=True)
            return
        first = self.poses[0]
        last = self.poses[-1]
        delta = [last[i] - first[i] for i in range(3)]
        displacement = math.sqrt(sum(value * value for value in delta))
        absolute_last = math.sqrt(sum(last[i] * last[i] for i in range(3)))
        dot = abs(sum(first[i] * last[i] for i in range(3, 7)))
        dot = max(-1.0, min(1.0, dot))
        angle_deg = math.degrees(2.0 * math.acos(dot))
        print(f"odometry_messages={len(self.poses)}", flush=True)
        print(f"covered_data_seconds={self.stamps[-1] - self.stamps[0]:.3f}", flush=True)
        maximum = max(math.dist(p[:3], first[:3]) for p in self.poses)
        print(f"max_displacement_from_first_m={maximum:.6f}", flush=True)
        print("Relative stationary drift only; not absolute measurement accuracy.", flush=True)
        print(f"first_position={first[:3]}", flush=True)
        print(f"last_position={last[:3]}", flush=True)
        print(f"first_to_last_displacement_m={displacement:.6f}", flush=True)
        print(f"last_distance_from_origin_m={absolute_last:.6f}", flush=True)
        print(f"first_to_last_rotation_deg={angle_deg:.6f}", flush=True)


def main():
    rclpy.init()
    node = DriftCheck()
    try:
        end_time = time.monotonic() + 65.0
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
