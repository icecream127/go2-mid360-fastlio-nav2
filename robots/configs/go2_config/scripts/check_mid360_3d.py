#!/usr/bin/env python3
"""Validate simulated MID-360 and FAST-LIO 3D outputs."""

import math
import struct
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CheckMid3603D(Node):
    def __init__(self):
        super().__init__("check_mid360_3d")
        self.raw_result = None
        self.custom_result = None
        self.registered_result = None
        self.create_subscription(
            PointCloud2, "/livox/lidar", self.on_raw, qos_profile_sensor_data
        )
        self.create_subscription(
            CustomMsg, "/livox/lidar_custom", self.on_custom, 10
        )
        self.create_subscription(
            PointCloud2, "/cloud_registered", self.on_registered, 10
        )

    @staticmethod
    def cloud_summary(message):
        if message.width == 0:
            return (0, math.nan, math.nan)
        field_offsets = {field.name: field.offset for field in message.fields}
        if not all(name in field_offsets for name in ("x", "y", "z")):
            return (message.width, math.nan, math.nan)
        z_values = []
        for index in range(message.width * message.height):
            base = index * message.point_step
            z_value = struct.unpack_from(
                "<f", message.data, base + field_offsets["z"]
            )[0]
            if math.isfinite(z_value):
                z_values.append(z_value)
        if not z_values:
            return (message.width, math.nan, math.nan)
        return (message.width, min(z_values), max(z_values))

    def on_raw(self, message):
        self.raw_result = self.cloud_summary(message)

    def on_registered(self, message):
        self.registered_result = self.cloud_summary(message)

    def on_custom(self, message):
        if not message.points:
            self.custom_result = (0, 0, 0)
            return
        offsets = [point.offset_time for point in message.points]
        self.custom_result = (message.point_num, min(offsets), max(offsets))


def main():
    rclpy.init()
    node = CheckMid3603D()
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline and (
        node.raw_result is None
        or node.custom_result is None
        or node.registered_result is None
    ):
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f"raw_xyz={node.raw_result}")
    print(f"custom_count_and_offset_ns={node.custom_result}")
    print(f"fastlio_registered_xyz={node.registered_result}")
    success = (
        node.raw_result is not None
        and node.raw_result[0] > 100
        and node.raw_result[2] - node.raw_result[1] > 0.5
        and node.custom_result is not None
        and node.custom_result[0] > 100
        and node.custom_result[2] > node.custom_result[1]
        and node.registered_result is not None
        and node.registered_result[0] > 100
    )
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
