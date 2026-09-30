#!/usr/bin/env python3
"""Isolated ROS-topic test: zero stamp, simulated pause/jump, reset and read-only config."""
import os
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, Bool, String
from tf2_msgs.msg import TFMessage


def main():
    rng = np.random.default_rng(90)
    faces = []
    for axis in range(3):
        for side in (-2, 2):
            points = rng.uniform(-2, 2, (700, 3)); points[:, axis] = side; faces.append(points)
    points = np.concatenate(faces).astype('<f4')
    prefix = '/gicp_runtime_' + str(os.getpid())
    with tempfile.TemporaryDirectory(prefix='gicp-runtime-') as directory:
        pcd = Path(directory) / 'map.pcd'
        pcd.write_bytes((f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                         f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n').encode() + points.tobytes())
        executable = Path(get_package_prefix('go2_real_localization')) / 'lib/go2_real_localization/gicp_localizer'
        command = [str(executable), '--ros-args', '-r', f'__ns:={prefix}', '-p', f'map_path:={pcd}',
                   '-p', 'use_sim_time:=true', '-p', 'initial_pose_enabled:=true', '-p', 'voxel_size:=0.1',
                   '-p', 'registration_period:=0.1', '-p', 'max_input_age:=0.8', '-p', 'transform_timeout:=1.0']
        for source, target in [('/cloud_registered_body', '/cloud'), ('/Odometry', '/odom'),
                               ('/initialpose', '/initial'), ('/clock', '/clock'), ('/tf', '/tf')]:
            command += ['-r', f'{source}:={prefix}{target}']
        with open(Path(directory) / 'node.log', 'w+') as log:
            process = subprocess.Popen(command, stdout=log, stderr=log)
            rclpy.init(); node = rclpy.create_node('runtime_check_' + str(os.getpid()))
            clock_pub = node.create_publisher(Clock, prefix + '/clock', 10)
            cloud_pub = node.create_publisher(PointCloud2, prefix + '/cloud', 10)
            odom_pub = node.create_publisher(Odometry, prefix + '/odom', 10)
            initial_pub = node.create_publisher(PoseWithCovarianceStamped, prefix + '/initial', 10)
            observed = {'valid': False, 'pose': None, 'status': '', 'tf_count': 0}
            latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(Bool, prefix + '/gicp/valid', lambda m: observed.update(valid=m.data), latched)
            node.create_subscription(String, prefix + '/gicp/status', lambda m: observed.update(status=m.data), latched)
            node.create_subscription(PoseStamped, prefix + '/gicp/pose', lambda m: observed.update(pose=m), 10)
            node.create_subscription(TFMessage, prefix + '/tf', lambda m: observed.update(tf_count=observed['tf_count'] + len(m.transforms)), 100)
            sim = 0.0

            def stamp(seconds):
                return rclpy.time.Time(nanoseconds=round(seconds * 1e9)).to_msg()

            def pair(seconds):
                header = Header(stamp=stamp(seconds), frame_id='livox_frame')
                cloud_pub.publish(create_cloud_xyz32(header, points))
                odom = Odometry(); odom.header = Header(stamp=header.stamp, frame_id='odom')
                odom.child_frame_id = 'livox_frame'; odom.pose.pose.orientation.w = 1.0; odom_pub.publish(odom)

            def pump(seconds, advance=True, publish=False, fixed_stamp=None, predicate=None):
                nonlocal sim
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    if process.poll() is not None: raise AssertionError('Node exited')
                    if advance: sim += 0.02
                    clock_pub.publish(Clock(clock=stamp(sim)))
                    if publish: pair(sim if fixed_stamp is None else fixed_stamp)
                    for _ in range(10): rclpy.spin_once(node, timeout_sec=0.001)
                    time.sleep(0.015)
                    if predicate and predicate(): return True
                return predicate is None or predicate()

            try:
                # Discover endpoints with clock held at zero; the first cloud really has stamp 0.
                pump(2.0, advance=False, publish=True, fixed_stamp=0)
                assert pump(0.7, publish=True, fixed_stamp=0,
                            predicate=lambda: observed['valid'] and observed['pose'] is not None), observed
                assert observed['pose'] is not None and observed['pose'].header.stamp.sec == 0
                assert observed['pose'].header.stamp.nanosec == 0
                print('PASS zero timestamp scan accepted')
                pump(0.2, advance=False)
                count = observed['tf_count']
                pump(1.2, advance=False)
                assert observed['valid'] and observed['tf_count'] == count, observed
                print('PASS paused ROS clock pauses scheduling and TTL')

                client = node.create_client(SetParameters, prefix + '/gicp_localizer/set_parameters')
                assert client.wait_for_service(timeout_sec=3)
                for name, value in [('voxel_size', 0.02), ('use_sim_time', False)]:
                    request = SetParameters.Request(parameters=[Parameter(name, value=value).to_parameter_msg()])
                    future = client.call_async(request)
                    assert pump(2, advance=False, predicate=future.done)
                    assert not future.result().results[0].successful
                print('PASS runtime configuration changes rejected explicitly')

                sim += 2
                assert pump(1, predicate=lambda: not observed['valid']), observed
                print('PASS forward ROS time jump expires localization')
                assert pump(2, publish=True, predicate=lambda: observed['valid']), observed
                print('PASS fresh input recovers localization')
                sim = 0
                assert pump(0.5, advance=False, publish=True, predicate=lambda: not observed['valid']), observed
                pump(0.3, publish=True)
                assert not observed['valid'], observed
                seed = PoseWithCovarianceStamped(); seed.header.frame_id = 'map'; seed.pose.pose.orientation.w = 1.0
                initial_pub.publish(seed)
                assert pump(2, publish=True, predicate=lambda: observed['valid']), observed
                print('PASS backward clock jump invalidates and requires a new seed')
            except Exception:
                log.flush(); log.seek(0); print(log.read()); raise
            finally:
                process.terminate()
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
                node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
