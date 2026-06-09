#!/usr/bin/env python3
"""
/lidar/points_world 누적해서 단일 PCD/NPY 로 저장 (매 실행마다 덮어쓰기).

저장 경로 (host 에선 /home/ercuam/A4VAI-Algorithms-ROS2/logs/):
  /home/user/a4vai_ws/logs/accumulated_pc.pcd  (ASCII PCD, CloudCompare 등)
  /home/user/a4vai_ws/logs/accumulated_pc.npy  (numpy, plot 용)

Voxel downsampling: 같은 0.5m 셀 내 점은 1개만 유지 (메모리 절약).
"""

import argparse
import os
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


OUT_PCD = '/home/user/a4vai_ws/logs/accumulated_pc.pcd'
OUT_NPY = '/home/user/a4vai_ws/logs/accumulated_pc.npy'
VOXEL = 0.5    # 0.5m grid


class PcAccumulator(Node):
    def __init__(self, voxel):
        super().__init__('pc_accumulator')
        self.voxel = float(voxel)
        self.cells = set()              # voxel keys (ix, iy, iz)
        self.pts = []                   # list of (x, y, z)
        self.cnt_msg = 0
        self.create_subscription(PointCloud2, '/lidar/points_world',
                                 self._cb, 10)
        self.create_timer(5.0, self._stat)
        self.create_timer(10.0, self._autosave)   # SIGKILL 대비 주기 save
        self.get_logger().info(
            f'pc_accumulator started. voxel={self.voxel}m  '
            f'out: {OUT_PCD}, {OUT_NPY}')

    def _autosave(self):
        if self.pts:
            self.save()

    def _cb(self, msg):
        self.cnt_msg += 1
        try:
            offsets = {f.name: f.offset for f in msg.fields if f.name in ('x','y','z')}
            if len(offsets) < 3:
                return
            dt = np.dtype({'names':['x','y','z'], 'formats':['<f4','<f4','<f4'],
                           'offsets':[offsets['x'], offsets['y'], offsets['z']],
                           'itemsize': msg.point_step})
            s = np.frombuffer(msg.data, dtype=dt)
            pts = np.stack([s['x'], s['y'], s['z']], axis=1).astype(np.float32)
            valid = np.isfinite(pts).all(axis=1)
            pts = pts[valid]
            if len(pts) == 0:
                return
            # voxel dedup
            keys = np.floor(pts / self.voxel).astype(np.int32)
            for i, k in enumerate(map(tuple, keys)):
                if k not in self.cells:
                    self.cells.add(k)
                    self.pts.append(pts[i])
        except Exception as e:
            self.get_logger().warn(f'cb err: {e}')

    def _stat(self):
        self.get_logger().info(
            f'msgs={self.cnt_msg}  unique voxels={len(self.cells)}')

    def save(self):
        if not self.pts:
            self.get_logger().warn('no points to save')
            return
        arr = np.asarray(self.pts, dtype=np.float32)
        os.makedirs(os.path.dirname(OUT_NPY), exist_ok=True)
        np.save(OUT_NPY, arr)
        self.get_logger().info(f'saved npy: {OUT_NPY} ({len(arr)} pts)')
        # PCD ASCII
        with open(OUT_PCD, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {len(arr)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(arr)}\n')
            f.write('DATA ascii\n')
            for p in arr:
                f.write(f'{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n')
        self.get_logger().info(f'saved pcd: {OUT_PCD}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--voxel', type=float, default=VOXEL)
    args, _ = p.parse_known_args()

    rclpy.init()
    node = PcAccumulator(args.voxel)

    def handler(sig, frame):
        node.get_logger().info(f'signal {sig} received, saving...')
        node.save()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass


if __name__ == '__main__':
    main()
