#!/usr/bin/env python3
"""
pf_attitude_smoother.py — pf 출력 자세 명령 SLERP LPF (band-aid for reWP transient).

흐름:
  /pf_att_2_control (raw — pf 가 발행, reWP 마다 transient 진동 포함)
       ↓  SLERP LPF (시간 상수 τ)
  /pf_att_2_control_filtered  (orchestrator 가 remap 으로 구독)

SLERP LPF:
  새 입력 q_in 도착 시 dt 측정 → α = 1 - exp(-dt/τ)
  q_filtered = SLERP(q_filtered_prev, q_in, α)
  thrust_body 도 동일 시간 상수로 LPF.

옵션:
  --tau-att       자세 LPF 시간 상수 s (기본 0.3)
  --tau-thrust    thrust LPF 시간 상수 s (기본 0.2)
  --in-topic      입력 토픽 (기본 /pf_att_2_control)
  --out-topic     출력 토픽 (기본 /pf_att_2_control_filtered)

Usage:
  python3 pf_attitude_smoother.py
  python3 pf_attitude_smoother.py --tau-att 0.5

ROS2 launcher 에서 orchestrator 가 filtered 토픽 구독하도록 remap:
  ros2 run algorithm_test path_following_bridge_test \\
      --ros-args -r /pf_att_2_control:=/pf_att_2_control_filtered
"""

import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleAttitudeSetpoint, PathFollowingAttCmd
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock


def slerp(q_a, q_b, t):
    """Spherical linear interpolation between unit quaternions q_a, q_b at parameter t∈[0,1].
    q_a, q_b: 4-tuples (w, x, y, z). Returns 4-tuple."""
    a = np.array(q_a, dtype=float)
    b = np.array(q_b, dtype=float)
    # ensure unit
    na = np.linalg.norm(a); nb = np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return q_b
    a /= na; b /= nb
    dot = float(np.dot(a, b))
    # 가장 짧은 회전 방향 선택 (q 와 -q 는 같은 회전이라 sign flip 가능)
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        # 거의 같은 방향 → 선형 보간 + 정규화
        out = a + t * (b - a)
        out /= np.linalg.norm(out)
        return tuple(out.tolist())
    theta_0 = math.acos(dot)
    sin_t0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_t = math.sin(theta)
    s_a = math.cos(theta) - dot * sin_t / sin_t0
    s_b = sin_t / sin_t0
    out = s_a * a + s_b * b
    out /= np.linalg.norm(out)
    return tuple(out.tolist())


class PFAttitudeSmoother(Node):
    def __init__(self, tau_att, tau_thr, in_topic, out_topic):
        super().__init__('pf_attitude_smoother')
        self.tau_att = float(tau_att)
        self.tau_thr = float(tau_thr)

        self.q_filt = None     # last filtered quaternion (w, x, y, z)
        self.thr_filt = None   # last filtered thrust_body (3-vec)
        self.last_t = None

        self.create_subscription(VehicleAttitudeSetpoint, in_topic, self._cb, 10)
        self.pub = self.create_publisher(VehicleAttitudeSetpoint, out_topic, 10)
        # PX4 SLERP fusion 용 PathFollowingAttCmd 변환 발행
        self.px4_pub = self.create_publisher(
            PathFollowingAttCmd, '/vehicle1/fmu/in/path_following_att_cmd',
            qos_profile_sensor_data)

        self.cnt_in = 0
        self.cnt_out = 0
        self.t_start = time.time()
        self.create_timer(2.0, self._stats)

        self.get_logger().info(
            f'PFAttitudeSmoother: tau_att={self.tau_att}s tau_thr={self.tau_thr}s '
            f'in={in_topic} out={out_topic}'
        )

    def _cb(self, msg):
        self.cnt_in += 1
        now = time.time()
        q_in = (float(msg.q_d[0]), float(msg.q_d[1]),
                float(msg.q_d[2]), float(msg.q_d[3]))
        thr_in = (float(msg.thrust_body[0]), float(msg.thrust_body[1]),
                  float(msg.thrust_body[2]))
        q_in_valid   = not any(math.isnan(v) for v in q_in)
        thr_in_valid = not any(math.isnan(v) for v in thr_in)

        # 필터 초기화: 처음 valid input 받을 때 한 번
        if self.q_filt is None:
            if not q_in_valid or not thr_in_valid:
                return  # 아직 valid 한 거 없음
            self.q_filt = q_in
            self.thr_filt = thr_in
            self.last_t = now
        else:
            dt = max(0.001, now - self.last_t)
            self.last_t = now
            # NaN 입력은 필터 갱신 skip — 이전 valid filtered 값 그대로 사용
            if q_in_valid:
                alpha_att = 1.0 - math.exp(-dt / max(self.tau_att, 1e-6))
                alpha_att = min(max(alpha_att, 0.0), 1.0)
                self.q_filt = slerp(self.q_filt, q_in, alpha_att)
            if thr_in_valid:
                alpha_thr = 1.0 - math.exp(-dt / max(self.tau_thr, 1e-6))
                alpha_thr = min(max(alpha_thr, 0.0), 1.0)
                self.thr_filt = tuple(
                    self.thr_filt[i] + alpha_thr * (thr_in[i] - self.thr_filt[i])
                    for i in range(3)
                )

        out = VehicleAttitudeSetpoint()
        ysmr = float(msg.yaw_sp_move_rate)
        out.yaw_sp_move_rate = ysmr if not math.isnan(ysmr) else 0.0
        out.q_d[0] = self.q_filt[0]
        out.q_d[1] = self.q_filt[1]
        out.q_d[2] = self.q_filt[2]
        out.q_d[3] = self.q_filt[3]
        out.thrust_body[0] = self.thr_filt[0]
        out.thrust_body[1] = self.thr_filt[1]
        out.thrust_body[2] = self.thr_filt[2]
        self.pub.publish(out)

        # PX4 SLERP fusion 입력 토픽으로도 발행 (PathFollowingAttCmd 타입)
        px4_msg = PathFollowingAttCmd()
        px4_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        px4_msg.pf_q_cmd = [
            float(self.q_filt[0]), float(self.q_filt[1]),
            float(self.q_filt[2]), float(self.q_filt[3])
        ]
        px4_msg.pf_thrust_cmd = [
            float(self.thr_filt[0]), float(self.thr_filt[1]), float(self.thr_filt[2])
        ]
        self.px4_pub.publish(px4_msg)
        self.cnt_out += 1

    def _stats(self):
        t = time.time() - self.t_start
        self.get_logger().info(
            f'[stats] t={t:.0f}s in={self.cnt_in} out={self.cnt_out}'
        )


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--tau-att',    type=float, default=0.3)
    p.add_argument('--tau-thrust', type=float, default=0.2)
    p.add_argument('--in-topic',   default='/pf_att_2_control')
    p.add_argument('--out-topic',  default='/pf_att_2_control_filtered')
    args, _ = p.parse_known_args()

    rclpy.init()
    node = PFAttitudeSmoother(args.tau_att, args.tau_thrust, args.in_topic, args.out_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
