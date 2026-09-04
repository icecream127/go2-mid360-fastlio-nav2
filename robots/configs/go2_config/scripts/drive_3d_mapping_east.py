#!/usr/bin/env python3
"""Continue the simulation-time mapping route into the east side."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class EastMappingRoute(Node):
    def __init__(self):
        super().__init__("drive_3d_mapping_east")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def segment(self, label, duration, linear=0.0, angular=0.0):
        print(label, flush=True)
        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            current_ns = self.get_clock().now().nanoseconds
            if current_ns > start_ns and (current_ns - start_ns) / 1.0e9 >= duration:
                break
            self.publish(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = EastMappingRoute()
    try:
        deadline = time.monotonic() + 20.0
        while node.get_clock().now().nanoseconds == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds == 0:
            raise RuntimeError("Simulation clock did not start")
        node.segment("1/6 back away from center wall", 4.0, linear=-0.22)
        node.segment("2/6 turn south", 3.9, angular=-0.40)
        node.segment("3/6 move below center wall", 10.6, linear=0.22)
        node.segment("4/6 turn east", 3.9, angular=0.40)
        node.segment("5/6 cross to east side", 8.0, linear=0.22)
        node.segment("6/6 scan east side", 4.0, angular=0.40)
        print("East mapping route complete", flush=True)
    finally:
        for _ in range(20):
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
