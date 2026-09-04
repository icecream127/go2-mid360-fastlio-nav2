#!/usr/bin/env python3
"""Publish planar simulation odometry from Gazebo's ground-truth pose."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GroundTruthOdom(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_odom")
        self.initial_pose = None
        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, "/odom/ground_truth", self.on_ground_truth, 10
        )

    def on_ground_truth(self, source: Odometry) -> None:
        position = source.pose.pose.position
        orientation = source.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        if self.initial_pose is None:
            self.initial_pose = (position.x, position.y, yaw)

        initial_x, initial_y, initial_yaw = self.initial_pose
        world_dx = position.x - initial_x
        world_dy = position.y - initial_y
        cos_initial = math.cos(initial_yaw)
        sin_initial = math.sin(initial_yaw)
        x = cos_initial * world_dx + sin_initial * world_dy
        y = -sin_initial * world_dx + cos_initial * world_dy
        relative_yaw = normalize_angle(yaw - initial_yaw)

        half_yaw = 0.5 * relative_yaw
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        # Gazebo P3D reports world-frame linear velocity. Rotate it into the
        # planar robot frame expected by nav_msgs/Odometry.
        velocity = source.twist.twist
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_vx = cos_yaw * velocity.linear.x + sin_yaw * velocity.linear.y
        body_vy = -sin_yaw * velocity.linear.x + cos_yaw * velocity.linear.y

        odom = Odometry()
        odom.header.stamp = source.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = body_vx
        odom.twist.twist.linear.y = body_vy
        odom.twist.twist.angular.z = velocity.angular.z
        odom.pose.covariance[0] = 1.0e-4
        odom.pose.covariance[7] = 1.0e-4
        odom.pose.covariance[35] = 1.0e-4
        odom.twist.covariance[0] = 1.0e-3
        odom.twist.covariance[7] = 1.0e-3
        odom.twist.covariance[35] = 1.0e-3
        self.publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
