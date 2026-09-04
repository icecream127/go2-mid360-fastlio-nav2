#!/usr/bin/env python3
"""Drive a short mapping route using simulation time."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MappingRoute(Node):
    def __init__(self):
        super().__init__("drive_3d_mapping_route")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def run_segment(self, duration, linear=0.0, angular=0.0):
        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            current_ns = self.get_clock().now().nanoseconds
            if current_ns > start_ns and (current_ns - start_ns) / 1.0e9 >= duration:
                break
            self.publish(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = MappingRoute()
    try:
        print("Waiting for simulation clock and subscribers...")
        deadline = time.monotonic() + 20.0
        while node.get_clock().now().nanoseconds == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds == 0:
            raise RuntimeError("Simulation clock did not start")
        print("Segment 1/4: forward")
        node.run_segment(4.0, linear=0.22)
        print("Segment 2/4: turn left")
        node.run_segment(2.5, angular=0.40)
        print("Segment 3/4: forward")
        node.run_segment(4.0, linear=0.22)
        print("Segment 4/4: turn right")
        node.run_segment(2.5, angular=-0.40)
        print("Mapping route complete")
    finally:
        for _ in range(20):
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
