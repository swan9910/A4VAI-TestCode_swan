#!/usr/bin/env python3
# wp_arrival_monitor.py
# 사용자가 입력한 GPS 점들에 드론이 도달할 때마다 현재 GPS 출력.
#
# 입력: input_gps.txt — "LABEL lat lon" per line
# 구독: /vehicle1/fmu/out/vehicle_global_position (px4_msgs/VehicleGlobalPosition)
# 출력: 각 target 도달 시 한 번만 stdout 으로 출력

import argparse
import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleGlobalPosition

R_EARTH = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(a))


class ArrivalMonitor(Node):
    def __init__(self, targets, threshold_m):
        super().__init__("wp_arrival_monitor")
        self.targets = targets  # list of (label, lat, lon)
        self.reached = [False] * len(targets)
        self.threshold = threshold_m
        self.t0 = time.time()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            VehicleGlobalPosition,
            "/vehicle1/fmu/out/vehicle_global_position",
            self.cb,
            qos,
        )
        self.get_logger().info(
            f"monitor 시작: {len(targets)} 점, threshold={threshold_m}m"
        )
        for lab, la, lo in targets:
            self.get_logger().info(f"  · {lab}: ({la}, {lo})")

    def cb(self, msg):
        lat, lon, alt = msg.lat, msg.lon, msg.alt
        for i, (label, t_lat, t_lon) in enumerate(self.targets):
            if self.reached[i]:
                continue
            d = haversine_m(lat, lon, t_lat, t_lon)
            if d < self.threshold:
                self.reached[i] = True
                t = time.time() - self.t0
                msg_str = (
                    f"[arrival] {label} 도달 (t={t:.1f}s)  "
                    f"현재 GPS: lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}m  "
                    f"목표까지={d:.2f}m"
                )
                print(msg_str, flush=True)
                self.get_logger().info(msg_str)
        if all(self.reached):
            self.get_logger().info("모든 target 도달, 종료")
            rclpy.shutdown()


def load_targets(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            out.append((parts[0], float(parts[1]), float(parts[2])))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-gps", required=True)
    ap.add_argument("--threshold", type=float, default=5.0)
    args = ap.parse_args()

    targets = load_targets(args.input_gps)
    if not targets:
        print(f"input_gps 비어있음: {args.input_gps}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = ArrivalMonitor(targets, args.threshold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
