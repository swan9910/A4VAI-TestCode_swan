#!/usr/bin/env python3
"""
드론 NED 위치 + fusion_weight 를 10Hz UDP 송신.
usage: 통합 launcher 안에서 backgound 로 호출
"""
import argparse, socket
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition, FusionWeight

class Streamer(Node):
    def __init__(self, ip, port, rate):
        super().__init__('flight_streamer')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ip, port)
        self.ned_x = 0.0; self.ned_y = 0.0; self.fw = 0.0
        self.create_subscription(VehicleLocalPosition,
            '/vehicle1/fmu/out/vehicle_local_position', self._cb_pos, qos_profile_sensor_data)
        self.create_subscription(FusionWeight,
            '/vehicle1/fmu/in/fusion_weight', self._cb_fw, qos_profile_sensor_data)
        self.create_timer(1.0/rate, self._tick)
        self.get_logger().info(f'streaming POS → {ip}:{port} @ {rate}Hz')
    def _cb_pos(self, msg):
        self.ned_x = float(msg.x); self.ned_y = float(msg.y)
    def _cb_fw(self, msg):
        self.fw = float(msg.fusion_weight)
    def _tick(self):
        try:
            self.sock.sendto(f'POS|{self.ned_x:.3f},{self.ned_y:.3f},{self.fw:.3f}'.encode(), self.addr)
        except Exception: pass

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ip', default='100.68.0.70')
    p.add_argument('--port', type=int, default=45680)
    p.add_argument('--rate', type=float, default=10.0)
    a, _ = p.parse_known_args()
    rclpy.init()
    n = Streamer(a.ip, a.port, a.rate)
    try: rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
