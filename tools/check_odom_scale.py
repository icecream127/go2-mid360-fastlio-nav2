#!/usr/bin/env python3
"""Compare CHAMP odometry against Gazebo ground truth with a short drive."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomScaleCheck(Node):
    def __init__(self) -> None:
        super().__init__("go2_odom_scale_check")
        self.odom = None
        self.ground_truth = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            Odometry, "/odom/ground_truth", self._on_ground_truth, 10
        )
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_odom(self, message: Odometry) -> None:
        self.odom = message

    def _on_ground_truth(self, message: Odometry) -> None:
        self.ground_truth = message

    def publish_velocity(self, speed: float) -> None:
        message = Twist()
        message.linear.x = speed
        self.cmd_vel.publish(message)


def position(message: Odometry) -> tuple[float, float]:
    return message.pose.pose.position.x, message.pose.pose.position.y


def distance(start: tuple[float, float], end: tuple[float, float]) -> float:
    return math.hypot(end[0] - start[0], end[1] - start[1])


def spin_for(node: OdomScaleCheck, duration: float, speed: float = 0.0) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        node.publish_velocity(speed)
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--sample-timeout", type=float, default=20.0)
    parser.add_argument("--min-ratio", type=float, default=0.70)
    parser.add_argument("--max-ratio", type=float, default=1.30)
    args = parser.parse_args()

    rclpy.init(args=[])
    node = OdomScaleCheck()

    try:
        sample_deadline = time.monotonic() + args.sample_timeout
        while (
            node.odom is None or node.ground_truth is None
        ) and time.monotonic() < sample_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.odom is None or node.ground_truth is None:
            print("[FAIL] /odom or /odom/ground_truth is not publishing")
            return 2

        odom_start = position(node.odom)
        truth_start = position(node.ground_truth)

        spin_for(node, args.duration, args.speed)
        spin_for(node, 1.0, 0.0)

        odom_end = position(node.odom)
        truth_end = position(node.ground_truth)
        odom_distance = distance(odom_start, odom_end)
        truth_distance = distance(truth_start, truth_end)
        ratio = odom_distance / truth_distance if truth_distance > 0.01 else None

        print(f"Command       : {args.speed:.2f} m/s for {args.duration:.1f} s")
        print(f"Odom start/end: {odom_start} -> {odom_end}")
        print(f"Truth start/end: {truth_start} -> {truth_end}")
        print(f"Odom distance : {odom_distance:.3f} m")
        print(f"Truth distance: {truth_distance:.3f} m")
        if ratio is None:
            print("Scale ratio   : inconclusive (truth distance is below 0.01 m)")
            print("[INCONCLUSIVE] Increase speed or duration and run again")
            return 3

        print(f"Scale ratio   : {ratio:.3f}")

        passed = args.min_ratio <= ratio <= args.max_ratio
        print(
            f"[{'PASS' if passed else 'FAIL'}] odometry scale is within "
            f"{args.min_ratio:.2f} .. {args.max_ratio:.2f}"
        )
        return 0 if passed else 1
    finally:
        for _ in range(10):
            node.publish_velocity(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
