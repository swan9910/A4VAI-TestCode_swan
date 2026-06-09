#!/usr/bin/env python3
"""
mode_switcher — strict switch + wide hysteresis (single 3D distance)

설계 원칙:
  - **switch-style**: fusion=0 (CA) or fusion=pf_cap (PF), 중간값 절대 안 거침
    → pf NDO state 가 partial-blend 에 의한 corruption 안 받음
  - **wide hysteresis**: enter / exit 거리 차이 크게 (예: 4m / 8m) → chatter 불가
  - **single distance metric**: 3D 가장 가까운 lidar 점 (omnidirectional)
    → cone/sphere 두 metric 사이 race 없음

[입력]
  /lidar/points_world   (PointCloud2, world frame)
  /ego_odom             (Odometry)

[출력]
  /vehicle1/fmu/in/fusion_weight  (FusionWeight)

옵션:
  --dist-ca-enter   : CA 진입 거리 (m, 기본 4.0) — dist < 이면 CA
  --dist-pf-enter   : PF 복귀 거리 (m, 기본 8.0) — dist > 이면 PF
  --rate            : 발행 주기 (Hz, 기본 20)
  --manual          : 디버그용 fusion 고정 (0 or 1)
  --pf-cap          : PF 모드 fusion 값 (기본 0.5)
  --ramp-time       : mode 전환 시 fusion 보간 시간 (s, 기본 0 = strict instant)
  --csv-log         : 거리/모드 csv
"""

import argparse
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from px4_msgs.msg import FusionWeight


SELF_RETURN_SKIP = 0.3   # m, 가까운 self-return / noise 무시


class ModeSwitcher(Node):
    def __init__(self, dist_ca, dist_pf, rate_hz, manual_mode, pf_cap, ramp_time=0.0, csv_log=None):
        super().__init__('mode_switcher')

        assert dist_ca < dist_pf, 'dist_ca_enter must be < dist_pf_enter for hysteresis'
        self.dist_ca = float(dist_ca)
        self.dist_pf = float(dist_pf)
        self.manual_mode = manual_mode
        self.pf_cap = float(pf_cap)
        self.ramp_time = float(ramp_time)
        self.csv_log = csv_log
        # ramp state
        self.t_switch = None      # 마지막 mode 전환 시각
        self.w_from = 0.0         # 전환 시작 fusion 값
        self.w_to = 0.0           # 전환 목표 fusion 값
        if csv_log:
            with open(csv_log, 'w', newline='') as f:
                csv.writer(f).writerow([
                    't_sec', 'fusion_weight', 'current_mode',
                    'min_dist', 'drone_x', 'drone_y', 'drone_z',
                ])

        # 상태
        self.drone_pos = np.zeros(3)
        self.latest_pts = None
        self.last_pts_time = 0.0
        self.current_mode = 0          # 시작: CA 안전
        self.fusion_weight = 0.0
        self.t_node_start = time.time()

        # 구독
        self.create_subscription(PointCloud2, '/lidar/points_world', self._cb_lidar, 10)
        self.create_subscription(Odometry, '/ego_odom', self._cb_odom, 10)

        # 발행
        self.fusion_pub = self.create_publisher(
            FusionWeight, '/vehicle1/fmu/in/fusion_weight', qos_profile_sensor_data)

        # 타이머
        self.create_timer(1.0/rate_hz, self._tick)
        self.create_timer(2.0, self._stats)
        self.t_start = time.time()
        self.tick_cnt = 0
        self.switch_cnt = 0

        if manual_mode is not None:
            self.get_logger().info(
                f'[MANUAL] mode={manual_mode}  pf_cap={self.pf_cap}')
        else:
            self.get_logger().info(
                f'strict switch  dist_ca={self.dist_ca}m  dist_pf={self.dist_pf}m  '
                f'pf_cap={self.pf_cap}  rate={rate_hz}Hz')

    def _cb_lidar(self, msg):
        offsets = {f.name: f.offset for f in msg.fields if f.name in ('x','y','z')}
        if len(offsets) < 3:
            return
        dt = np.dtype({'names': ['x','y','z'], 'formats': ['<f4','<f4','<f4'],
                       'offsets': [offsets['x'], offsets['y'], offsets['z']],
                       'itemsize': msg.point_step})
        s = np.frombuffer(msg.data, dtype=dt)
        pts = np.stack([s['x'], s['y'], s['z']], axis=1).astype(np.float32)
        valid = np.isfinite(pts).all(axis=1)
        self.latest_pts = pts[valid]
        self.last_pts_time = time.time()

    def _cb_odom(self, msg):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

    def _min_dist(self):
        """drone 주위 lidar 점들 중 가장 가까운 3D 거리 — slab ±SLAB_HALF m 안 점만 (지면 제외).
        하강 시 drone alt 가 작아지면 slab 도 자연 따라감 → ground 가 slab 안에 들어와 안전.
        lidar 데이터 없거나 stale 이면 None 반환 — mode 전환 안 함."""
        SLAB_HALF = 2.0   # drone z ±2m slab
        if self.latest_pts is None or len(self.latest_pts) == 0:
            return None
        if (time.time() - self.last_pts_time) > 1.0:
            return None
        pts = self.latest_pts
        drone_pos = self.drone_pos.astype(np.float32)
        # slab 필터: |pt.z - drone.z| < SLAB_HALF
        slab_mask = np.abs(pts[:, 2] - drone_pos[2]) < SLAB_HALF
        if not slab_mask.any():
            return float('inf')   # slab 안 점 없음 = open space
        pts_slab = pts[slab_mask]
        d = np.linalg.norm(pts_slab - drone_pos, axis=1)
        d_safe = d[d > SELF_RETURN_SKIP]
        if len(d_safe) == 0:
            return float('inf')
        return float(d_safe.min())

    def _tick(self):
        self.tick_cnt += 1
        prev_mode = self.current_mode

        d = self._min_dist()

        if self.manual_mode is not None:
            self.current_mode = self.manual_mode
        elif d is None:
            # lidar 데이터 없음 — 모드 전환 안 함 (안전 측면, 초기 race 방지)
            pass
        else:
            # strict switch with wide hysteresis
            if self.current_mode == 1:               # PF → CA 검사
                if d < self.dist_ca:
                    self.current_mode = 0
                    self.switch_cnt += 1
                    self.get_logger().info(
                        f'★ dist={d:.2f}m < {self.dist_ca}m → CA mode (fusion=0)')
            else:                                     # CA → PF 검사
                if d > self.dist_pf:
                    self.current_mode = 1
                    self.switch_cnt += 1
                    self.get_logger().info(
                        f'☆ dist={d:.2f}m > {self.dist_pf}m → PF mode (fusion={self.pf_cap})')

        # mode 전환 시 ramp 시작점 저장
        if prev_mode != self.current_mode:
            self.t_switch = time.time()
            self.w_from = self.fusion_weight
            self.w_to = self.pf_cap if self.current_mode == 1 else 0.0

        # weight 계산
        target = self.pf_cap if self.current_mode == 1 else 0.0
        if self.ramp_time <= 0 or self.t_switch is None:
            self.fusion_weight = target
        else:
            elapsed = time.time() - self.t_switch
            if elapsed >= self.ramp_time:
                self.fusion_weight = self.w_to
            else:
                a = elapsed / self.ramp_time
                # smoothstep 으로 ease-in-out
                a = a * a * (3 - 2 * a)
                self.fusion_weight = self.w_from + a * (self.w_to - self.w_from)

        msg = FusionWeight()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.fusion_weight = float(self.fusion_weight)
        self.fusion_pub.publish(msg)

        if self.csv_log:
            t = time.time() - self.t_node_start
            if d is None:
                d_val = -2.0
            elif math.isinf(d):
                d_val = -1.0
            else:
                d_val = round(d, 3)
            with open(self.csv_log, 'a', newline='') as f:
                csv.writer(f).writerow([
                    round(t, 4), round(self.fusion_weight, 4),
                    self.current_mode, d_val,
                    round(self.drone_pos[0], 3),
                    round(self.drone_pos[1], 3),
                    round(self.drone_pos[2], 3),
                ])

    def _stats(self):
        t = time.time() - self.t_start
        d = self._min_dist()
        if d is None:
            d_str = 'no_lidar'
        elif math.isinf(d):
            d_str = 'inf'
        else:
            d_str = f'{d:.2f}'
        self.get_logger().info(
            f'[stats] t={t:.0f}s mode={"PF" if self.current_mode==1 else "CA"} '
            f'fusion={self.fusion_weight:.2f} dist={d_str}m switches={self.switch_cnt}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dist-ca-enter', type=float, default=4.0,
                        help='dist 이 아래면 CA mode 진입')
    parser.add_argument('--dist-pf-enter', type=float, default=8.0,
                        help='dist 이 위면 PF mode 진입 (hysteresis 위해 ca-enter 보다 크게)')
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--manual', type=int, default=None, choices=[None, 0, 1])
    parser.add_argument('--pf-cap', type=float, default=0.5)
    parser.add_argument('--ramp-time', type=float, default=0.0,
                        help='mode 전환 시 fusion 보간 시간 (s, 0=strict instant)')
    parser.add_argument('--csv-log', type=str, default=None)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ModeSwitcher(
        dist_ca=args.dist_ca_enter,
        dist_pf=args.dist_pf_enter,
        rate_hz=args.rate,
        manual_mode=args.manual,
        pf_cap=args.pf_cap,
        ramp_time=args.ramp_time,
        csv_log=args.csv_log,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
