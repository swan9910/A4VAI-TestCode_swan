#!/usr/bin/env python3
"""
fusion_weight 구독 -> UDP 로 mode 문자열 발송 (1Hz).
fusion > 0.5 : "pf"
fusion <= 0.5: "ca"
"""
import argparse, socket
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import FusionWeight

class ModeBroadcaster(Node):
    def __init__(self, ip, port, rate):
        super().__init__('mode_broadcaster')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ip, port)
        self.fusion = 0.0
        self.create_subscription(FusionWeight, '/vehicle1/fmu/in/fusion_weight',
                                 self._cb, qos_profile_sensor_data)
        self.create_timer(1.0/rate, self._tick)
        self.get_logger().info(f'broadcasting -> {ip}:{port}  rate={rate}Hz')
    def _cb(self, msg):
        self.fusion = float(msg.fusion_weight)
    def _tick(self):
        mode = 'pf' if self.fusion > 0.5 else 'ca'
        try:
            self.sock.sendto(f'mode:{mode}\n'.encode(), self.addr)
        except Exception as e:
            self.get_logger().warn(f'sendto fail: {e}')

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ip', default='100.68.0.70')
    p.add_argument('--port', type=int, default=45678)
    p.add_argument('--rate', type=float, default=1.0)
    a, _ = p.parse_known_args()
    rclpy.init()
    n = ModeBroadcaster(a.ip, a.port, a.rate)
    try: rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
