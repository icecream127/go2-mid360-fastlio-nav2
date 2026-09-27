import time
from datetime import datetime
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = rclpy.create_node('save_live_map_snapshot')
    received = []
    sub = node.create_subscription(PointCloud2, '/Laser_map', lambda m: received.append(m) if not received else None, 1)
    node.declare_parameter('output_dir', str(Path.home() / 'go2_maps'))
    node.declare_parameter('timeout_sec', 45.0)
    deadline = time.monotonic() + float(node.get_parameter('timeout_sec').value)
    print('Waiting for /Laser_map: saves the accumulated registered scans published so far.', flush=True)
    try:
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not received:
            raise RuntimeError('No accumulated /Laser_map received; no file saved')
        m = received[0]
        fields = {f.name: f for f in m.fields}
        names = [n for n in ('x', 'y', 'z', 'intensity') if n in fields]
        if not all(n in names for n in ('x', 'y', 'z')):
            raise RuntimeError('Missing XYZ')
        if any(fields[n].datatype != 7 or fields[n].count != 1 for n in names):
            raise RuntimeError('Expected scalar float32 fields')
        dtype = np.dtype({'names': names, 'formats': [('>' if m.is_bigendian else '<') + 'f4'] * len(names),
                          'offsets': [fields[n].offset for n in names], 'itemsize': m.point_step})
        data = np.ndarray((m.height, m.width), dtype=dtype, buffer=m.data, strides=(m.row_step, m.point_step))
        points = np.column_stack([data[n].reshape(-1) for n in names]).astype('<f4')
        points = points[np.isfinite(points[:, :3]).all(axis=1)]
        if not len(points):
            raise RuntimeError('Empty map; not saved')
        path = Path(node.get_parameter('output_dir').value).expanduser() / ('real_room_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.pcd')
        path.parent.mkdir(parents=True, exist_ok=True)
        header = '\n'.join(['# .PCD v0.7', 'VERSION 0.7', 'FIELDS ' + ' '.join(names),
            'SIZE ' + ' '.join(['4'] * len(names)), 'TYPE ' + ' '.join(['F'] * len(names)),
            'COUNT ' + ' '.join(['1'] * len(names)), f'WIDTH {len(points)}', 'HEIGHT 1',
            'VIEWPOINT 0 0 0 1 0 0 0', f'POINTS {len(points)}', 'DATA binary', ''])
        with path.open('xb') as out:
            out.write(header.encode('ascii'))
            out.write(points.tobytes())
        assert path.stat().st_size == len(header.encode('ascii')) + points.nbytes
        print(f'SAVED {path}\nPOINTS {len(points)}\nBYTES {path.stat().st_size}\nFRAME {m.header.frame_id}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
