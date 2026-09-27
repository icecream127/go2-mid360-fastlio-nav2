#!/usr/bin/env python3
"""Bounded integration check. Uses synthetic geometry, never the lidar or motors."""
import os
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String, Bool
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy


def transform(x, y, z, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    result[:3, 3] = [x, y, z]
    return result


def main():
    rng = np.random.default_rng(2026)
    pieces = []
    for axis, value in [(0, -3), (0, 4), (1, -2), (1, 5), (2, -1), (2, 3)]:
        points = rng.uniform([-3, -2, -1], [4, 5, 3], (700, 3))
        points[:, axis] = value
        pieces.append(points)
    pieces.append(rng.normal([1.4, 2.1, 0.8], [0.3, 0.4, 0.5], (700, 3)))
    reference = np.concatenate(pieces).astype(np.float32)
    map_body = transform(1.0, -0.4, 0.2, 0.2)
    odom_body = transform(0.3, 0.2, 0.1, -0.15)
    body_points = (reference - map_body[:3, 3]) @ map_body[:3, :3]
    with tempfile.TemporaryDirectory(prefix='gicp-test-') as directory:
        path = Path(directory) / 'reference.pcd'
        header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  f'WIDTH {len(reference)}\nHEIGHT 1\nPOINTS {len(reference)}\nDATA binary\n')
        path.write_bytes(header.encode() + reference.astype('<f4').tobytes())
        exe = Path(get_package_prefix('go2_real_localization')) / 'lib/go2_real_localization/gicp_localizer'
        params = Path(get_package_share_directory('go2_real_localization')) / 'config/gicp.yaml'
        with open(Path(directory) / 'node.log', 'w+') as log:
            process = subprocess.Popen([str(exe), '--ros-args', '--params-file', str(params),
                                        '-p', f'map_path:={path}', '-p', 'initial_pose_enabled:=true',
                                        '-p', 'initial_pose:=[1.12, -0.32, 0.23, 0.0, 0.0, 0.25]',
                                        '-p', 'registration_period:=0.3'], stdout=log, stderr=log)
            rclpy.init()
            node = rclpy.create_node('gicp_synthetic_check')
            cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', 5)
            odom_pub = node.create_publisher(Odometry, '/Odometry', 5)
            observed = {'pose': None, 'status': '', 'valid': False, 'tf': None}
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(PoseStamped, '/gicp/pose', lambda m: observed.update(pose=m), 5)
            node.create_subscription(String, '/gicp/status', lambda m: observed.update(status=m.data), latched)
            node.create_subscription(Bool, '/gicp/valid', lambda m: observed.update(valid=m.data), latched)
            def on_tf(msg):
                for t in msg.transforms:
                    if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                        observed['tf'] = t
            node.create_subscription(TFMessage, '/tf', on_tf, 10)
            try:
                def run_until(predicate, seconds, bad=False, publish=True):
                    deadline = time.monotonic() + seconds
                    next_pub = 0
                    while time.monotonic() < deadline:
                        if process.poll() is not None:
                            raise RuntimeError('GICP exited unexpectedly')
                        if publish and time.monotonic() > next_pub:
                            stamp = node.get_clock().now().to_msg()
                            odom = Odometry(); odom.header = Header(stamp=stamp, frame_id='odom')
                            odom.child_frame_id = 'livox_frame'
                            p = odom.pose.pose
                            p.position.x, p.position.y, p.position.z = odom_body[:3, 3].tolist()
                            p.orientation.z = float(np.sin(-0.15 / 2)); p.orientation.w = float(np.cos(-0.15 / 2))
                            points = body_points + (100.0 if bad else 0.0)
                            cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='livox_frame'), points))
                            odom_pub.publish(odom)
                            next_pub = time.monotonic() + 0.2
                        rclpy.spin_once(node, timeout_sec=0.05)
                        if predicate():
                            return
                    raise AssertionError(f'Timeout: {observed}')

                run_until(lambda: observed['pose'] is not None and observed['valid'] and observed['tf'] is not None, 25)
                p = observed['pose'].pose
                error = np.linalg.norm(np.array([p.position.x, p.position.y, p.position.z]) - map_body[:3, 3])
                yaw = 2 * np.arctan2(p.orientation.z, p.orientation.w)
                assert error < 0.05 and abs(yaw - 0.2) < 0.03, (error, yaw)
                print(f'PASS GICP pose: translation error={error:.6f}m, yaw error={abs(yaw - 0.2):.6f}rad')
                expected = map_body @ np.linalg.inv(odom_body)
                actual = observed['tf'].transform
                tf_error = np.linalg.norm(np.array([actual.translation.x, actual.translation.y, actual.translation.z]) - expected[:3, 3])
                tf_yaw = 2 * np.arctan2(actual.rotation.z, actual.rotation.w)
                assert tf_error < 0.05 and abs(tf_yaw - 0.35) < 0.03, (tf_error, tf_yaw)
                print(f'PASS map->odom composition: translation error={tf_error:.6f}m')
                run_until(lambda: not observed['valid'] and ('REJECTED' in observed['status'] or 'NOT_CONVERGED' in observed['status']), 15, bad=True)
                print('PASS unrelated cloud rejected:', observed['status'])
                run_until(lambda: observed['valid'], 15)
                print('PASS recovery on valid cloud')
                run_until(lambda: not observed['valid'] and observed['status'] in ('STALE_INPUT', 'LOCALIZATION_EXPIRED'), 8, publish=False)
                print('PASS stale input invalidated')
            except Exception:
                log.flush(); log.seek(0); print(log.read()); raise
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
                node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
