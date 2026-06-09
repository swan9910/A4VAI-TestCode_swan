#!/usr/bin/env python3
"""
pf 노드 (node_att_ctrl) 가 valid attitude 계산하려면 필요한 것:
  1) /local_waypoint_setpoint_to_PF      (LocalWaypointSetpoint)
  2) /controller_heartbeat               (Bool True, 1Hz)
  3) /path_planning_heartbeat            (Bool True, 1Hz)
  4) /collision_avoidance_heartbeat      (Bool True, 1Hz)

path_following_bridge_test 없이 통합에서 pf 살리는 용도.

wp QoS: TRANSIENT_LOCAL latch → pf 늦게 sub 해도 받음.
heartbeat 는 1Hz 주기 발행.
"""

import argparse
import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from custom_msgs.msg import LocalWaypointSetpoint
from std_msgs.msg import Bool


def load_wp_csv(path):
    xs, ys, zs = [], [], []
    with open(path) as f:
        reader = csv.DictReader(f, skipinitialspace=True)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            zs.append(float(row['z']))
    return xs, ys, zs


class WpPublisher(Node):
    def __init__(self, wp_path, repeat, period):
        super().__init__('wp_publisher')
        self.xs, self.ys, self.zs = load_wp_csv(wp_path)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            LocalWaypointSetpoint, '/local_waypoint_setpoint_to_PF', qos)
        # heartbeat publishers (default QoS — pf sub depth=10 RELIABLE)
        self.hb_ctrl  = self.create_publisher(Bool, '/controller_heartbeat',           10)
        self.hb_plan  = self.create_publisher(Bool, '/path_planning_heartbeat',        10)
        self.hb_ca    = self.create_publisher(Bool, '/collision_avoidance_heartbeat',  10)
        self.repeat_left = repeat
        self.get_logger().info(
            f'wp loaded: x={self.xs} y={self.ys} z={self.zs}  repeat={repeat} period={period}s'
        )
        # 첫 발행 (latch)
        self._publish_once()
        if repeat > 1:
            self.create_timer(period, self._tick)
        # heartbeat 1Hz 무한 발행
        self.create_timer(1.0, self._hb_tick)

    def _publish_once(self):
        msg = LocalWaypointSetpoint()
        msg.path_planning_complete = True
        msg.waypoint_x = self.xs
        msg.waypoint_y = self.ys
        msg.waypoint_z = self.zs
        self.pub.publish(msg)
        self.get_logger().info(f'published wp ({self.repeat_left} left)')
        self.repeat_left -= 1

    def _tick(self):
        if self.repeat_left > 0:
            self._publish_once()

    def _hb_tick(self):
        m = Bool()
        m.data = True
        self.hb_ctrl.publish(m)
        self.hb_plan.publish(m)
        self.hb_ca.publish(m)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--wp-csv', type=str, required=True)
    p.add_argument('--repeat', type=int, default=5)
    p.add_argument('--period', type=float, default=1.0)
    args, _ = p.parse_known_args()

    rclpy.init()
    node = WpPublisher(args.wp_csv, args.repeat, args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
