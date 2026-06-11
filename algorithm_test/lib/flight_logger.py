#!/usr/bin/env python3
"""
ego-planner 비행 분석용 로그 생성기

기록 토픽:
  /vehicle1/fmu/out/vehicle_local_position  (PX4 NED state, 100Hz)
  /ego_odom                                  (ENU odom from offboard.py)
  /vehicle1/fmu/out/vehicle_attitude         (PX4 quaternion)
  /planning/pos_cmd                          (ego-planner trajectory cmd, 100Hz)
  /vehicle1/fmu/in/trajectory_setpoint       (what offboard.py 가 PX4 로 보내는 것)
  /lidar/points_world                        (point count, 10Hz)
  /move_base_simple/goal                     (goal events)

CSV 컬럼:
  t                            time since logger start (sec)
  ned_x, ned_y, ned_z          PX4 NED position
  ned_vx, ned_vy, ned_vz       PX4 NED velocity
  enu_x, enu_y, enu_z          ENU position from /ego_odom
  qw, qx, qy, qz               drone attitude (ENU world)
  roll_deg, pitch_deg, yaw_deg
  ego_x, ego_y, ego_z          ego-planner pos_cmd (ENU)
  ego_vx, ego_vy, ego_vz
  traj_x, traj_y, traj_z       PX4 trajectory_setpoint (NED)
  lidar_pts                    /lidar/points_world point count
  reset_xy, reset_z            EKF reset counters
  goal_x, goal_y, goal_z       most recent goal (ENU)

[사용법]
  python3 flight_logger.py --duration 60 --csv /home/user/flight_logs/flight.csv
"""

import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import (
    VehicleLocalPosition, VehicleAttitude, TrajectorySetpoint,
    VehicleAttitudeSetpoint, FusionWeight, PathFollowingAttCmd,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import LocalWaypointSetpoint
import numpy as np


def quat_to_rpy(qw, qx, qy, qz):
    sinr = 2*(qw*qx + qy*qz)
    cosr = 1 - 2*(qx*qx + qy*qy)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2*(qw*qy - qz*qx)))
    pitch = math.asin(sinp)
    siny = 2*(qw*qz + qx*qy)
    cosy = 1 - 2*(qy*qy + qz*qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class FlightLogger(Node):
    def __init__(self, duration, csv_path):
        super().__init__('flight_logger')
        self.t_start = time.time()
        self.duration = duration
        self.csv_path = csv_path

        # latest data
        self.ned_pos = (0.0, 0.0, 0.0)
        self.ned_vel = (0.0, 0.0, 0.0)
        self.reset_xy = 0
        self.reset_z = 0
        # GPS (vehicle_local_position 의 ref_lat/lon/alt + NED 로 계산)
        self.gps_lat = float('nan')
        self.gps_lon = float('nan')
        self.gps_alt = float('nan')
        self.enu_pos = (0.0, 0.0, 0.0)
        self.quat = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
        self.have_local_pos = False   # 첫 vehicle_local_position 수신 전엔 row 안 씀
        self.ego_cmd_pos = (float('nan'),) * 3
        self.ego_cmd_vel = (float('nan'),) * 3
        self.traj_setpt = (float('nan'),) * 3
        self.traj_setpt_vel = (float('nan'),) * 3
        self.lidar_pts = 0
        self.goal = (float('nan'),) * 3
        self.cnt_ego = 0
        self.cnt_traj = 0
        self.cnt_lidar = 0
        self.cnt_goal = 0
        # SLERP 분석용 추가 데이터
        self.pf_q = (float('nan'),) * 4   # /pf_att_2_control q_d
        self.pf_thrust = (float('nan'),) * 3
        self.fusion_w = float('nan')
        self.pf_cmd_q = (float('nan'),) * 4  # /vehicle1/fmu/in/path_following_att_cmd
        self.pf_cmd_thrust = (float('nan'),) * 3
        self.att_sp_q = (float('nan'),) * 4  # /vehicle1/fmu/in/vehicle_attitude_setpoint
        self.cnt_pf = 0
        self.cnt_fw = 0
        self.cnt_pf_cmd = 0
        self.cnt_att_sp = 0
        # NDO observer state (pf 발행: GPR/in/dbl_Q6 = [t, out_NDO[0], out_NDO[1], out_NDO[2]])
        self.ndo = (float('nan'),) * 3
        self.cnt_ndo = 0
        # MPPI 출력 (진동 source 분석용)
        self.mppi_out = (float('nan'),) * 3   # [Ax, eta, calc_time_ms]
        self.cnt_mppi = 0
        # 회피 분석 추가 데이터
        self.planner_wp_count = 0                        # simple_planner 가 보낸 wp 개수
        self.planner_wp_first = (float('nan'),) * 2      # NED (north, east) 첫 wp
        self.planner_wp_last  = (float('nan'),) * 2      # NED (north, east) 마지막 wp
        self.cnt_planner_wp   = 0
        self.heading_wp_idx   = -1                       # pf 가 현재 추종 중인 wp idx
        self.lidar_nearest_dist = float('nan')           # 가장 가까운 obstacle horizontal dist (drone alt ±1m)
        self.lidar_nearest_az   = float('nan')           # 그 obstacle 의 방위 (rad, 0 = north 향)
        # 3D 전역 nearest (mode_switcher 와 동일 metric)
        self.lidar_3d_dist      = float('nan')           # 3D 거리
        self.lidar_3d_z         = float('nan')           # 그 점의 z (ENU alt) — 지면이면 0 근처

        # subscriptions
        self.create_subscription(
            VehicleLocalPosition, '/vehicle1/fmu/out/vehicle_local_position',
            self._cb_ned, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/ego_odom', self._cb_enu, 10)
        self.create_subscription(
            VehicleAttitude, '/vehicle1/fmu/out/vehicle_attitude',
            self._cb_att, qos_profile_sensor_data)
        self.create_subscription(
            PositionCommand, '/planning/pos_cmd', self._cb_ego, 10)
        self.create_subscription(
            TrajectorySetpoint, '/vehicle1/fmu/in/trajectory_setpoint',
            self._cb_traj, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/lidar/points_world', self._cb_lidar, 10)
        self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self._cb_goal, 1)
        # SLERP 분석용 추가 subscriptions
        self.create_subscription(
            VehicleAttitudeSetpoint, '/pf_att_2_control', self._cb_pf, 10)
        self.create_subscription(
            FusionWeight, '/vehicle1/fmu/in/fusion_weight',
            self._cb_fw, qos_profile_sensor_data)
        self.create_subscription(
            PathFollowingAttCmd, '/vehicle1/fmu/in/path_following_att_cmd',
            self._cb_pf_cmd, qos_profile_sensor_data)
        self.create_subscription(
            VehicleAttitudeSetpoint, '/vehicle1/fmu/in/vehicle_attitude_setpoint',
            self._cb_att_sp, qos_profile_sensor_data)
        # MPPI 출력 — 진동 source 분석용
        from std_msgs.msg import Float64MultiArray, Int32
        self.create_subscription(
            Float64MultiArray, 'MPPI/out/dbl_MPPI', self._cb_mppi,
            qos_profile_sensor_data)
        # 회피 분석 추가 subscriptions
        self.create_subscription(
            LocalWaypointSetpoint, '/local_waypoint_setpoint_to_PF',
            self._cb_planner_wp, 1)
        self.create_subscription(
            Int32, '/heading_waypoint_index', self._cb_heading_idx, 10)
        # NDO state from pf node (Float64MultiArray [t, ndo_x, ndo_y, ndo_z])
        self.create_subscription(
            Float64MultiArray, 'GPR/in/dbl_Q6', self._cb_ndo, 10)

        # CSV header
        with open(csv_path, 'w', newline='') as f:
            csv.writer(f).writerow([
                't',
                'ned_x', 'ned_y', 'ned_z',
                'ned_vx', 'ned_vy', 'ned_vz',
                'reset_xy', 'reset_z',
                'gps_lat', 'gps_lon', 'gps_alt',
                'enu_x', 'enu_y', 'enu_z',
                'qw', 'qx', 'qy', 'qz',
                'roll_deg', 'pitch_deg', 'yaw_deg',
                'ego_x', 'ego_y', 'ego_z',
                'ego_vx', 'ego_vy', 'ego_vz',
                'traj_x', 'traj_y', 'traj_z',
                'traj_vx', 'traj_vy', 'traj_vz',
                'lidar_pts',
                'goal_x', 'goal_y', 'goal_z',
                # SLERP 분석
                'pf_qw', 'pf_qx', 'pf_qy', 'pf_qz',
                'pf_thrust_x', 'pf_thrust_y', 'pf_thrust_z',
                'fusion_w',
                'pf_cmd_qw', 'pf_cmd_qx', 'pf_cmd_qy', 'pf_cmd_qz',
                'pf_cmd_thrust_x', 'pf_cmd_thrust_y', 'pf_cmd_thrust_z',
                'att_sp_qw', 'att_sp_qx', 'att_sp_qy', 'att_sp_qz',
                'mppi_Ax', 'mppi_eta', 'mppi_calc_ms',
                'planner_wp_n', 'planner_wp_first_x', 'planner_wp_first_y',
                'planner_wp_last_x', 'planner_wp_last_y',
                'heading_wp_idx',
                'lidar_near_dist', 'lidar_near_az',
                'lidar_3d_dist', 'lidar_3d_z',
                'ndo_x', 'ndo_y', 'ndo_z',
            ])

        # 50Hz 기록 + 종료 + 통계
        self.records = []
        self.create_timer(0.02, self._tick)
        self.create_timer(2.0, self._stats)
        self.create_timer(duration, self._stop)

        self.get_logger().info(
            f'flight_logger started, duration={duration}s, csv={csv_path}')

    def _cb_ned(self, msg):
        self.ned_pos = (msg.x, msg.y, msg.z)
        self.ned_vel = (msg.vx, msg.vy, msg.vz)
        self.reset_xy = int(msg.xy_reset_counter)
        self.reset_z = int(msg.z_reset_counter)
        # NED → GPS: ref_lat/ref_lon/ref_alt + msg.x(north) / msg.y(east) / msg.z(down)
        if msg.xy_global and msg.z_global:
            R = 6378137.0
            dlat = msg.x / R
            cos_ref = math.cos(math.radians(msg.ref_lat))
            dlon = msg.y / (R * cos_ref) if abs(cos_ref) > 1e-9 else 0.0
            self.gps_lat = msg.ref_lat + math.degrees(dlat)
            self.gps_lon = msg.ref_lon + math.degrees(dlon)
            self.gps_alt = msg.ref_alt - msg.z  # NED down → alt 위로 양수

    def _cb_enu(self, msg):
        p = msg.pose.pose.position
        self.enu_pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self.quat = (q.w, q.x, q.y, q.z)

    def _cb_att(self, msg):
        # PX4 NED quaternion (already published in NED)
        # store separately or override? Use ENU from /ego_odom for consistency.
        pass

    def _cb_ego(self, msg):
        self.cnt_ego += 1
        self.ego_cmd_pos = (msg.position.x, msg.position.y, msg.position.z)
        self.ego_cmd_vel = (msg.velocity.x, msg.velocity.y, msg.velocity.z)

    def _cb_traj(self, msg):
        self.cnt_traj += 1
        self.traj_setpt = (msg.position[0], msg.position[1], msg.position[2])
        self.traj_setpt_vel = (msg.velocity[0], msg.velocity[1], msg.velocity[2])

    def _cb_lidar(self, msg):
        self.cnt_lidar += 1
        self.lidar_pts = int(msg.width * msg.height)
        # 가장 가까운 obstacle (drone 고도 ±1m, 수평 거리)
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
            drone_z = self.enu_pos[2] if not math.isnan(self.enu_pos[2]) else 3.0
            in_alt = pts[np.abs(pts[:, 2] - drone_z) < 1.0]
            if len(in_alt) == 0:
                self.lidar_nearest_dist = float('nan')
                self.lidar_nearest_az = float('nan')
                return
            dxy = in_alt[:, :2] - np.array(self.enu_pos[:2])
            dist = np.linalg.norm(dxy, axis=1)
            idx_min = int(np.argmin(dist))
            self.lidar_nearest_dist = float(dist[idx_min])
            self.lidar_nearest_az = float(np.arctan2(dxy[idx_min, 1], dxy[idx_min, 0]))
            # 3D 전역 nearest (slab 필터 없음, 지면 포함)
            drone_pos = np.array(self.enu_pos, dtype=np.float32)
            d3 = np.linalg.norm(pts - drone_pos, axis=1)
            # self-return 제외 (0.3m 이하)
            mask3 = d3 > 0.3
            if mask3.any():
                d3_safe = d3[mask3]
                pts_safe = pts[mask3]
                idx3 = int(np.argmin(d3_safe))
                self.lidar_3d_dist = float(d3_safe[idx3])
                self.lidar_3d_z = float(pts_safe[idx3, 2])
        except Exception:
            pass

    def _cb_goal(self, msg):
        self.cnt_goal += 1
        self.goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.get_logger().info(
            f'★ Goal received: ({self.goal[0]:.2f}, {self.goal[1]:.2f}, {self.goal[2]:.2f})')

    def _cb_pf(self, msg):
        self.cnt_pf += 1
        self.pf_q = (msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3])
        self.pf_thrust = (msg.thrust_body[0], msg.thrust_body[1], msg.thrust_body[2])

    def _cb_fw(self, msg):
        self.cnt_fw += 1
        self.fusion_w = float(msg.fusion_weight)

    def _cb_pf_cmd(self, msg):
        self.cnt_pf_cmd += 1
        self.pf_cmd_q = (msg.pf_q_cmd[0], msg.pf_q_cmd[1], msg.pf_q_cmd[2], msg.pf_q_cmd[3])
        self.pf_cmd_thrust = (msg.pf_thrust_cmd[0], msg.pf_thrust_cmd[1], msg.pf_thrust_cmd[2])

    def _cb_att_sp(self, msg):
        self.cnt_att_sp += 1
        self.att_sp_q = (msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3])

    def _cb_mppi(self, msg):
        self.cnt_mppi += 1
        if len(msg.data) >= 3:
            self.mppi_out = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def _cb_planner_wp(self, msg):
        self.cnt_planner_wp += 1
        n = len(msg.waypoint_x)
        self.planner_wp_count = n
        if n > 0:
            self.planner_wp_first = (float(msg.waypoint_x[0]),  float(msg.waypoint_y[0]))
            self.planner_wp_last  = (float(msg.waypoint_x[-1]), float(msg.waypoint_y[-1]))

    def _cb_ndo(self, msg):
        d = list(msg.data) if msg.data else []
        if len(d) >= 4:
            self.ndo = (float(d[1]), float(d[2]), float(d[3]))
            self.cnt_ndo += 1

    def _cb_heading_idx(self, msg):
        self.heading_wp_idx = int(msg.data)

    def _tick(self):
        t = time.time() - self.t_start
        roll, pitch, yaw = quat_to_rpy(*self.quat)
        self.records.append([
            round(t, 3),
            round(self.ned_pos[0], 4), round(self.ned_pos[1], 4), round(self.ned_pos[2], 4),
            round(self.ned_vel[0], 4), round(self.ned_vel[1], 4), round(self.ned_vel[2], 4),
            self.reset_xy, self.reset_z,
            round(self.gps_lat, 7) if not math.isnan(self.gps_lat) else 'nan',
            round(self.gps_lon, 7) if not math.isnan(self.gps_lon) else 'nan',
            round(self.gps_alt, 3) if not math.isnan(self.gps_alt) else 'nan',
            round(self.enu_pos[0], 4), round(self.enu_pos[1], 4), round(self.enu_pos[2], 4),
            round(self.quat[0], 6), round(self.quat[1], 6),
            round(self.quat[2], 6), round(self.quat[3], 6),
            round(math.degrees(roll), 3), round(math.degrees(pitch), 3),
            round(math.degrees(yaw), 3),
            round(self.ego_cmd_pos[0], 4), round(self.ego_cmd_pos[1], 4),
            round(self.ego_cmd_pos[2], 4),
            round(self.ego_cmd_vel[0], 4), round(self.ego_cmd_vel[1], 4),
            round(self.ego_cmd_vel[2], 4),
            round(self.traj_setpt[0], 4), round(self.traj_setpt[1], 4),
            round(self.traj_setpt[2], 4),
            round(self.traj_setpt_vel[0], 4), round(self.traj_setpt_vel[1], 4),
            round(self.traj_setpt_vel[2], 4),
            self.lidar_pts,
            round(self.goal[0], 4) if not math.isnan(self.goal[0]) else 'nan',
            round(self.goal[1], 4) if not math.isnan(self.goal[1]) else 'nan',
            round(self.goal[2], 4) if not math.isnan(self.goal[2]) else 'nan',
            # SLERP 분석
            round(self.pf_q[0], 6), round(self.pf_q[1], 6),
            round(self.pf_q[2], 6), round(self.pf_q[3], 6),
            round(self.pf_thrust[0], 4), round(self.pf_thrust[1], 4),
            round(self.pf_thrust[2], 4),
            round(self.fusion_w, 4) if not math.isnan(self.fusion_w) else 'nan',
            round(self.pf_cmd_q[0], 6), round(self.pf_cmd_q[1], 6),
            round(self.pf_cmd_q[2], 6), round(self.pf_cmd_q[3], 6),
            round(self.pf_cmd_thrust[0], 4), round(self.pf_cmd_thrust[1], 4),
            round(self.pf_cmd_thrust[2], 4),
            round(self.att_sp_q[0], 6), round(self.att_sp_q[1], 6),
            round(self.att_sp_q[2], 6), round(self.att_sp_q[3], 6),
            round(self.mppi_out[0], 6), round(self.mppi_out[1], 6),
            round(self.mppi_out[2], 4),
            self.planner_wp_count,
            round(self.planner_wp_first[0], 4) if not math.isnan(self.planner_wp_first[0]) else 'nan',
            round(self.planner_wp_first[1], 4) if not math.isnan(self.planner_wp_first[1]) else 'nan',
            round(self.planner_wp_last[0], 4) if not math.isnan(self.planner_wp_last[0]) else 'nan',
            round(self.planner_wp_last[1], 4) if not math.isnan(self.planner_wp_last[1]) else 'nan',
            self.heading_wp_idx,
            round(self.lidar_nearest_dist, 3) if not math.isnan(self.lidar_nearest_dist) else 'nan',
            round(self.lidar_nearest_az, 4) if not math.isnan(self.lidar_nearest_az) else 'nan',
            round(self.lidar_3d_dist, 3) if not math.isnan(self.lidar_3d_dist) else 'nan',
            round(self.lidar_3d_z, 3) if not math.isnan(self.lidar_3d_z) else 'nan',
            round(self.ndo[0], 4) if not math.isnan(self.ndo[0]) else 'nan',
            round(self.ndo[1], 4) if not math.isnan(self.ndo[1]) else 'nan',
            round(self.ndo[2], 4) if not math.isnan(self.ndo[2]) else 'nan',
        ])
        if len(self.records) >= 100:
            self._flush()

    def _flush(self):
        if not self.records:
            return
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerows(self.records)
        self.records.clear()

    def _stats(self):
        t = time.time() - self.t_start
        fw_str = f'{self.fusion_w:.3f}' if not math.isnan(self.fusion_w) else 'nan'
        self.get_logger().info(
            f'[t={t:.0f}s] ego={self.cnt_ego} traj={self.cnt_traj} '
            f'lidar={self.cnt_lidar} goal={self.cnt_goal} '
            f'pf={self.cnt_pf} fw={self.cnt_fw} pf_cmd={self.cnt_pf_cmd} '
            f'att_sp={self.cnt_att_sp}  '
            f'ENU=({self.enu_pos[0]:+.1f}, {self.enu_pos[1]:+.1f}, {self.enu_pos[2]:+.1f})  '
            f'fusion={fw_str}  reset_xy={self.reset_xy}')

    def _stop(self):
        self._flush()
        self.get_logger().info(f'=== {self.duration}s elapsed, stopping ===')
        raise SystemExit


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--csv', type=str, default='/home/user/flight_logs/flight.csv')
    args, _ = parser.parse_known_args()

    os.makedirs(os.path.dirname(args.csv), exist_ok=True)

    rclpy.init()
    node = FlightLogger(args.duration, args.csv)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node._flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
