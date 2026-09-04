#!/usr/bin/env python3

"""Fast planar MID360 simulation for static SDF worlds.

Gazebo Classic's ODE multiray sensor is prohibitively slow for dense 3D scans
under WSL.  This node parses static box and cylinder collisions from the active
world, ray-casts a horizontal scan analytically, and publishes both LaserScan
and PointCloud2 using the same interfaces as the MID360 navigation pipeline.
"""

import math
import struct
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def pose2d(element):
    text = element.findtext("pose", default="0 0 0 0 0 0")
    values = [float(value) for value in text.split()]
    values += [0.0] * (6 - len(values))
    return values[0], values[1], values[5]


def compose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    cosine, sine = math.cos(pyaw), math.sin(pyaw)
    return (
        px + cosine * cx - sine * cy,
        py + sine * cx + cosine * cy,
        pyaw + cyaw,
    )


class FastMid360Scan(Node):
    def __init__(self):
        super().__init__("fast_mid360_scan")
        self.declare_parameter("world_file", "")
        self.declare_parameter("frame_id", "mid360_link")
        self.declare_parameter("samples", 720)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("range_min", 0.30)
        self.declare_parameter("range_max", 15.0)
        self.declare_parameter("lidar_x", 0.20)

        self.frame_id = self.get_parameter("frame_id").value
        self.samples = int(self.get_parameter("samples").value)
        self.rate = float(self.get_parameter("rate").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.lidar_x = float(self.get_parameter("lidar_x").value)
        self.boxes = []
        self.circles = []
        self.robot_pose = None

        self._load_world(self.get_parameter("world_file").value)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/livox/lidar", qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            "/odom/ground_truth",
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.rate, self._publish_scan)
        self.get_logger().info(
            f"Loaded {len(self.boxes)} boxes and {len(self.circles)} cylinders"
        )

    def _load_world(self, path):
        if not path:
            self.get_logger().warning("No world file supplied; scans will be empty")
            return
        root = ET.parse(path).getroot()
        world = root.find("world")
        if world is None:
            raise RuntimeError(f"No <world> element in {path}")
        for model in world.findall("model"):
            if model.findtext("static", default="false").lower() != "true":
                continue
            model_pose = pose2d(model)
            for link in model.findall("link"):
                link_pose = compose(model_pose, pose2d(link))
                for collision in link.findall("collision"):
                    collision_pose = compose(link_pose, pose2d(collision))
                    geometry = collision.find("geometry")
                    if geometry is None:
                        continue
                    box = geometry.find("box")
                    cylinder = geometry.find("cylinder")
                    if box is not None:
                        size = [float(v) for v in box.findtext("size").split()]
                        self.boxes.append((*collision_pose, size[0], size[1]))
                    elif cylinder is not None:
                        radius = float(cylinder.findtext("radius"))
                        self.circles.append(
                            (collision_pose[0], collision_pose[1], radius)
                        )

    def _odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.robot_pose = (position.x, position.y, yaw)

    @staticmethod
    def _ray_box(origin, direction, box):
        bx, by, byaw, width, depth = box
        cosine, sine = math.cos(byaw), math.sin(byaw)
        dx, dy = origin[0] - bx, origin[1] - by
        local_origin = (cosine * dx + sine * dy, -sine * dx + cosine * dy)
        local_direction = (
            cosine * direction[0] + sine * direction[1],
            -sine * direction[0] + cosine * direction[1],
        )
        near, far = -math.inf, math.inf
        for value, slope, half_size in zip(
            local_origin, local_direction, (width * 0.5, depth * 0.5)
        ):
            if abs(slope) < 1e-12:
                if abs(value) > half_size:
                    return math.inf
                continue
            first = (-half_size - value) / slope
            second = (half_size - value) / slope
            near = max(near, min(first, second))
            far = min(far, max(first, second))
            if near > far:
                return math.inf
        if far < 0.0:
            return math.inf
        return near if near >= 0.0 else far

    @staticmethod
    def _ray_circle(origin, direction, circle):
        cx, cy, radius = circle
        ox, oy = origin[0] - cx, origin[1] - cy
        projection = ox * direction[0] + oy * direction[1]
        discriminant = projection * projection - (ox * ox + oy * oy - radius * radius)
        if discriminant < 0.0:
            return math.inf
        root = math.sqrt(discriminant)
        first, second = -projection - root, -projection + root
        if first >= 0.0:
            return first
        return second if second >= 0.0 else math.inf

    def _publish_scan(self):
        if self.robot_pose is None:
            return
        robot_x, robot_y, yaw = self.robot_pose
        origin = (
            robot_x + self.lidar_x * math.cos(yaw),
            robot_y + self.lidar_x * math.sin(yaw),
        )
        angle_min = -math.pi
        increment = 2.0 * math.pi / self.samples
        ranges = []
        points = []
        for index in range(self.samples):
            local_angle = angle_min + index * increment
            world_angle = yaw + local_angle
            direction = (math.cos(world_angle), math.sin(world_angle))
            distance = self.range_max
            for box in self.boxes:
                distance = min(distance, self._ray_box(origin, direction, box))
            for circle in self.circles:
                distance = min(distance, self._ray_circle(origin, direction, circle))
            if distance < self.range_min or distance >= self.range_max:
                ranges.append(math.inf)
                continue
            ranges.append(distance)
            points.append(
                (distance * math.cos(local_angle), distance * math.sin(local_angle), 0.0)
            )

        stamp = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (self.samples - 1) * increment
        scan.angle_increment = increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
        self.cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = FastMid360Scan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
