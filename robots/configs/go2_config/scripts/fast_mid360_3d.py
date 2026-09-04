#!/usr/bin/env python3
"""Lightweight 3D MID-360 simulation for static SDF worlds.

The node analytically intersects MID-360 rays with box, cylinder and ground
collisions.  It avoids Gazebo Classic's expensive ODE multiray sensor while
publishing both a standard PointCloud2 and timestamped Livox CustomMsg for
FAST-LIO.  A separate horizontal LaserScan is retained for Nav2.
"""

import math
import struct
import xml.etree.ElementTree as ET

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def pose_values(element):
    text_value = element.findtext("pose", default="0 0 0 0 0 0")
    values = [float(value) for value in text_value.split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])


def compose_planar(parent, child):
    px, py, pz, _, _, pyaw = parent
    cx, cy, cz, croll, cpitch, cyaw = child
    cosine, sine = math.cos(pyaw), math.sin(pyaw)
    return (
        px + cosine * cx - sine * cy,
        py + sine * cx + cosine * cy,
        pz + cz,
        croll,
        cpitch,
        pyaw + cyaw,
    )


def rotate_vector(quaternion, vector):
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


class FastMid3603D(Node):
    def __init__(self):
        super().__init__("fast_mid360_3d")
        self.declare_parameter("world_file", "")
        self.declare_parameter("frame_id", "mid360_link")
        self.declare_parameter("horizontal_samples", 180)
        self.declare_parameter("vertical_samples", 12)
        self.declare_parameter("scan_samples", 360)
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("range_min", 0.30)
        self.declare_parameter("range_max", 15.0)
        self.declare_parameter("vertical_min_deg", -7.22)
        self.declare_parameter("vertical_max_deg", 55.22)
        self.declare_parameter("lidar_x", 0.20)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_z", 0.1177)

        self.frame_id = self.get_parameter("frame_id").value
        self.horizontal_samples = int(self.get_parameter("horizontal_samples").value)
        self.vertical_samples = int(self.get_parameter("vertical_samples").value)
        self.scan_samples = int(self.get_parameter("scan_samples").value)
        self.rate = float(self.get_parameter("rate").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.vertical_min = math.radians(
            float(self.get_parameter("vertical_min_deg").value)
        )
        self.vertical_max = math.radians(
            float(self.get_parameter("vertical_max_deg").value)
        )
        self.lidar_offset = (
            float(self.get_parameter("lidar_x").value),
            float(self.get_parameter("lidar_y").value),
            float(self.get_parameter("lidar_z").value),
        )
        self.boxes = []
        self.cylinders = []
        self.planes = []
        self.robot_pose = None

        self._load_world(self.get_parameter("world_file").value)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/livox/lidar", qos_profile_sensor_data
        )
        self.custom_pub = self.create_publisher(
            CustomMsg, "/livox/lidar_custom", 20
        )
        self.create_subscription(
            Odometry,
            "/odom/ground_truth",
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.rate, self._publish)
        self.get_logger().info(
            "Loaded %d boxes, %d cylinders and %d planes; output is %dx%d at %.1f Hz"
            % (
                len(self.boxes),
                len(self.cylinders),
                len(self.planes),
                self.horizontal_samples,
                self.vertical_samples,
                self.rate,
            )
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
            model_pose = pose_values(model)
            for link in model.findall("link"):
                link_pose = compose_planar(model_pose, pose_values(link))
                for collision in link.findall("collision"):
                    collision_pose = compose_planar(link_pose, pose_values(collision))
                    geometry = collision.find("geometry")
                    if geometry is None:
                        continue
                    box = geometry.find("box")
                    cylinder = geometry.find("cylinder")
                    plane = geometry.find("plane")
                    if box is not None:
                        size = [float(value) for value in box.findtext("size").split()]
                        self.boxes.append((*collision_pose, *size))
                    elif cylinder is not None:
                        self.cylinders.append(
                            (
                                collision_pose[0],
                                collision_pose[1],
                                collision_pose[2],
                                float(cylinder.findtext("radius")),
                                float(cylinder.findtext("length")),
                            )
                        )
                    elif plane is not None:
                        size = [float(value) for value in plane.findtext("size").split()]
                        self.planes.append(
                            (collision_pose[0], collision_pose[1], collision_pose[2], *size)
                        )

    def _odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.robot_pose = (
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
        )

    @staticmethod
    def _ray_box(origin, direction, box):
        bx, by, bz, _, _, yaw, sx, sy, sz = box
        cosine, sine = math.cos(yaw), math.sin(yaw)
        dx, dy, dz = origin[0] - bx, origin[1] - by, origin[2] - bz
        local_origin = (cosine * dx + sine * dy, -sine * dx + cosine * dy, dz)
        local_direction = (
            cosine * direction[0] + sine * direction[1],
            -sine * direction[0] + cosine * direction[1],
            direction[2],
        )
        near, far = -math.inf, math.inf
        for value, slope, half_size in zip(
            local_origin, local_direction, (sx * 0.5, sy * 0.5, sz * 0.5)
        ):
            if abs(slope) < 1.0e-12:
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
    def _ray_cylinder(origin, direction, cylinder):
        cx, cy, cz, radius, length = cylinder
        ox, oy, oz = origin[0] - cx, origin[1] - cy, origin[2] - cz
        dx, dy, dz = direction
        candidates = []
        quadratic = dx * dx + dy * dy
        if quadratic > 1.0e-12:
            projection = ox * dx + oy * dy
            discriminant = projection * projection - quadratic * (
                ox * ox + oy * oy - radius * radius
            )
            if discriminant >= 0.0:
                root = math.sqrt(discriminant)
                for distance in (
                    (-projection - root) / quadratic,
                    (-projection + root) / quadratic,
                ):
                    hit_z = oz + distance * dz
                    if distance >= 0.0 and abs(hit_z) <= length * 0.5:
                        candidates.append(distance)
        if abs(dz) > 1.0e-12:
            for cap_z in (-length * 0.5, length * 0.5):
                distance = (cap_z - oz) / dz
                hit_x = ox + distance * dx
                hit_y = oy + distance * dy
                if distance >= 0.0 and hit_x * hit_x + hit_y * hit_y <= radius * radius:
                    candidates.append(distance)
        return min(candidates, default=math.inf)

    @staticmethod
    def _ray_plane(origin, direction, plane):
        px, py, pz, sx, sy = plane
        if abs(direction[2]) < 1.0e-12:
            return math.inf
        distance = (pz - origin[2]) / direction[2]
        if distance < 0.0:
            return math.inf
        hit_x = origin[0] + distance * direction[0]
        hit_y = origin[1] + distance * direction[1]
        if abs(hit_x - px) <= sx * 0.5 and abs(hit_y - py) <= sy * 0.5:
            return distance
        return math.inf

    def _trace(self, origin, direction):
        distance = self.range_max
        for box in self.boxes:
            distance = min(distance, self._ray_box(origin, direction, box))
        for cylinder in self.cylinders:
            distance = min(distance, self._ray_cylinder(origin, direction, cylinder))
        for plane in self.planes:
            distance = min(distance, self._ray_plane(origin, direction, plane))
        if distance < self.range_min or distance >= self.range_max:
            return math.inf
        return distance

    def _sensor_origin(self, position, quaternion):
        offset = rotate_vector(quaternion, self.lidar_offset)
        return (
            position[0] + offset[0],
            position[1] + offset[1],
            position[2] + offset[2],
        )

    def _publish(self):
        if self.robot_pose is None:
            return
        position, quaternion = self.robot_pose
        origin = self._sensor_origin(position, quaternion)
        points = []
        point_metadata = []
        total_rays = self.horizontal_samples * self.vertical_samples
        ray_index = 0
        for vertical_index in range(self.vertical_samples):
            ratio = vertical_index / max(1, self.vertical_samples - 1)
            elevation = self.vertical_min + ratio * (
                self.vertical_max - self.vertical_min
            )
            cos_elevation = math.cos(elevation)
            for horizontal_index in range(self.horizontal_samples):
                azimuth = -math.pi + 2.0 * math.pi * (
                    horizontal_index / self.horizontal_samples
                )
                local_direction = (
                    cos_elevation * math.cos(azimuth),
                    cos_elevation * math.sin(azimuth),
                    math.sin(elevation),
                )
                world_direction = rotate_vector(quaternion, local_direction)
                distance = self._trace(origin, world_direction)
                if math.isfinite(distance):
                    points.append(tuple(distance * value for value in local_direction))
                    point_metadata.append((ray_index, vertical_index))
                ray_index += 1

        stamp = self.get_clock().now().to_msg()
        scan_duration_ns = int(1.0e9 / self.rate)
        self._publish_cloud(stamp, points, point_metadata, total_rays, scan_duration_ns)
        self._publish_custom(stamp, points, point_metadata, total_rays, scan_duration_ns)
        self._publish_planar_scan(stamp, origin, quaternion)

    def _publish_cloud(self, stamp, points, metadata, total_rays, duration_ns):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 22
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        packed = []
        for point, (ray_index, vertical_index) in zip(points, metadata):
            offset_seconds = duration_ns * ray_index / max(1, total_rays - 1) / 1.0e9
            packed.append(
                struct.pack(
                    "<fffffH",
                    point[0],
                    point[1],
                    point[2],
                    100.0,
                    offset_seconds,
                    vertical_index,
                )
            )
        cloud.data = b"".join(packed)
        self.cloud_pub.publish(cloud)

    def _publish_custom(self, stamp, points, metadata, total_rays, duration_ns):
        message = CustomMsg()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.timebase = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        message.point_num = len(points)
        message.lidar_id = 1
        message.rsvd = [0, 0, 0]
        custom_points = []
        for point, (ray_index, vertical_index) in zip(points, metadata):
            custom = CustomPoint()
            custom.offset_time = int(
                duration_ns * ray_index / max(1, total_rays - 1)
            )
            custom.x, custom.y, custom.z = point
            custom.reflectivity = 100
            custom.tag = 0
            custom.line = vertical_index % 4
            custom_points.append(custom)
        message.points = custom_points
        self.custom_pub.publish(message)

    def _publish_planar_scan(self, stamp, origin, quaternion):
        angle_min = -math.pi
        increment = 2.0 * math.pi / self.scan_samples
        ranges = []
        for index in range(self.scan_samples):
            azimuth = angle_min + index * increment
            local_direction = (math.cos(azimuth), math.sin(azimuth), 0.0)
            world_direction = rotate_vector(quaternion, local_direction)
            ranges.append(self._trace(origin, world_direction))
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (self.scan_samples - 1) * increment
        scan.angle_increment = increment
        scan.scan_time = 1.0 / self.rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FastMid3603D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
