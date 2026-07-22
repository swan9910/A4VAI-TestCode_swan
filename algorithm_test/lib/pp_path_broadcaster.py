#!/usr/bin/env python3
"""
PSO 결과 (path_final.txt) 를 UDP 로 1회 전송.
usage: python3 pp_path_broadcaster.py --ip 100.68.0.70 --port 45680 \
        --path /home/user/a4vai_ws/pathplanning/pathplanning/Results_Images/path_final.txt
"""
import argparse, socket, numpy as np
p = argparse.ArgumentParser()
p.add_argument('--ip', default='100.68.0.70')
p.add_argument('--port', type=int, default=45680)
p.add_argument('--path', required=True)
a = p.parse_args()
d = np.loadtxt(a.path)          # cols: row, col, z (pixel)
tokens = [f'{r:.1f},{c:.1f}' for r, c, _ in d]
msg = ('PSO|' + ';'.join(tokens))
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(msg.encode(), (a.ip, a.port))
print(f'sent PSO path: {len(tokens)} pts → {a.ip}:{a.port} ({len(msg)} bytes)')
