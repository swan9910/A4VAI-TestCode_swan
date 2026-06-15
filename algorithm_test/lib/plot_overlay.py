#!/usr/bin/env python3
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import csv

HEIGHTMAP = '/home/user/a4vai_ws/pathplanning/pathplanning/map/IMG_0268_1000.png'
PATH_FINAL = '/home/user/a4vai_ws/pathplanning/pathplanning/Results_Images/path_final.txt'
OUT_PNG = '/home/user/a4vai_ws/pathplanning/pathplanning/Results_Images/path_overlay.png'

A, B = -0.0287, 0.6517
D, E =  0.8264, -0.0228
M_inv = np.linalg.inv(np.array([[A, B], [D, E]]))

def gz_to_pix_delta(dx, dy):
    return M_inv @ np.array([dx, dy])

def latest_csv():
    p = Path('/home/user/a4vai_ws/logs/flight_csv')
    return max(p.glob('*.csv'), key=lambda f: f.stat().st_mtime)

def load_csv(path):
    out = {'t':[], 'ned_x':[], 'ned_y':[], 'ned_z':[], 'fw':[]}
    with open(path) as f:
        for r in csv.DictReader(f):
            out['t'].append(float(r['t']))
            out['ned_x'].append(float(r['ned_x']))
            out['ned_y'].append(float(r['ned_y']))
            out['ned_z'].append(float(r['ned_z']))
            try: out['fw'].append(float(r['fusion_w']))
            except: out['fw'].append(float('nan'))
    return {k: np.array(v) for k, v in out.items()}

pso = np.loadtxt(PATH_FINAL)
pso_row, pso_col = pso[:, 0], pso[:, 1]
start_row, start_col = pso_row[0], pso_col[0]

csv_path = sys.argv[1] if len(sys.argv) > 1 else str(latest_csv())
print(f'csv: {csv_path}')
fl = load_csv(csv_path)

flight_row, flight_col = [], []
for nx, ny in zip(fl['ned_x'], fl['ned_y']):
    drow, dcol = gz_to_pix_delta(ny, -nx)
    flight_row.append(start_row + drow); flight_col.append(start_col + dcol)
flight_row = np.array(flight_row); flight_col = np.array(flight_col)

fw = fl['fw']
img = mpimg.imread(HEIGHTMAP)
fig, ax = plt.subplots(figsize=(12, 12))
ax.imshow(img, cmap='gray')
ax.plot(pso_col, pso_row, 'c--', linewidth=1.5, label='PSO path', alpha=0.8)
ax.plot(pso_col[0], pso_row[0], 'go', markersize=12, label='Start')
ax.plot(pso_col[-1], pso_row[-1], 'bo', markersize=12, label='Goal')
ca = fw <= 0.5; pf = fw > 0.5
ax.plot(flight_col[ca], flight_row[ca], '.', color='red', markersize=2, label='Flight (CA)', alpha=0.6)
ax.plot(flight_col[pf], flight_row[pf], '.', color='orange', markersize=2, label='Flight (PF)', alpha=0.6)
ax.plot(flight_col[-1], flight_row[-1], 'r*', markersize=20, label='Flight end')
ax.set_title(f'PSO path + actual flight overlay\n{Path(csv_path).name}')
ax.set_xlabel('col'); ax.set_ylabel('row')
ax.legend(loc='upper right')
plt.savefig(OUT_PNG, dpi=120, bbox_inches='tight')
print(f'saved {OUT_PNG}')
