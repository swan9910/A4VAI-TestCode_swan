#!/usr/bin/env python3
"""Integration flight plot — 4x2 with shaded mode backgrounds + velocity + stats."""
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

csv_path = sys.argv[1]
out_png = sys.argv[2]
goal = (float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else (300, 0, 20)

df = pd.read_csv(csv_path)
df['alt'] = -df.ned_z
# keep all alt values (drone may fly below origin in some scenarios)

# Auto-crop to arrival: first time drone is within tolerance of goal, +5s margin.
# If never reached, keep full data.
tol_xy = 5.0; tol_z = 3.0
arrived = (df.enu_x.between(goal[0]-tol_xy, goal[0]+tol_xy) &
           df.enu_y.between(goal[1]-tol_xy, goal[1]+tol_xy) &
           df.alt.between(goal[2]-tol_z, goal[2]+tol_z))
if arrived.any():
    t_arrived = df.loc[arrived.idxmax(), 't']
    t_cut = t_arrived + 5.0
    df = df[df.t <= t_cut].reset_index(drop=True)
    print(f'arrived @ t={t_arrived:.1f}s, cropped to t<={t_cut:.1f}s')

# velocity in ENU (vx=east, vy=north, vz=up from NED)
df['enu_vx'] = df.ned_vy if 'ned_vy' in df.columns else 0.0
df['enu_vy'] = df.ned_vx if 'ned_vx' in df.columns else 0.0
df['enu_vz'] = -df.ned_vz if 'ned_vz' in df.columns else 0.0

# mode classification
def mode_label(w):
    if np.isnan(w): return 'no'
    if w <= 0.02: return 'CA'
    if w >= 0.4:  return 'PF'
    return 'tr'

mode_colors = {'CA': '#ffc8c8', 'PF': '#c8f0d4', 'tr': '#fff0c0', 'no': '#e0e0e0'}
line_colors = {'CA': '#e54f4f', 'PF': '#3ab363', 'tr': '#f0a020', 'no': '#888888'}

df['mode'] = df.fusion_w.apply(mode_label)
colors_line = [line_colors[m] for m in df['mode']]

# mode bands (start, end, mode_label)
def make_bands():
    bands = []
    if len(df) == 0: return bands
    cur = df['mode'].iloc[0]; ss = df.t.iloc[0]
    for i, m in enumerate(df['mode']):
        if m != cur:
            bands.append((ss, df.t.iloc[i], cur))
            cur = m; ss = df.t.iloc[i]
    bands.append((ss, df.t.iloc[-1], cur))
    return bands

bands = make_bands()

def shade_bg(ax):
    for s, e, m in bands:
        ax.axvspan(s, e, color=mode_colors[m], alpha=0.5, zorder=0)

def colored_line(ax, x, y, lw=1.7):
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, colors=colors_line[:-1], linewidths=lw, alpha=0.95, zorder=3)
    ax.add_collection(lc)

# stats
ca_dur = sum((e-s) for s,e,m in bands if m=='CA')
pf_dur = sum((e-s) for s,e,m in bands if m=='PF')
tot = df.t.max() - df.t.min()
err_x = abs(df.enu_x.iloc[-1] - goal[0])
err_y = abs(df.enu_y.iloc[-1] - goal[1])
err_z = abs(df.alt.iloc[-1] - goal[2])

y_abs_max = max(abs(df.enu_y.max()), abs(df.enu_y.min()), 3)

# === plot ===
fig, axes = plt.subplots(4, 2, figsize=(18, 14))

# (0,0) Top-down
ax = axes[0, 0]
colored_line(ax, df.enu_x.values, df.enu_y.values, lw=2.5)
ax.scatter([0], [0], s=300, c='#2060ff', marker='o', edgecolors='k', lw=1.5, zorder=5)
ax.scatter([goal[0]], [goal[1]], s=400, c='#ffd900', marker='*', edgecolors='k', lw=1.5, zorder=5)
x_lo = min(df.enu_x.min(), goal[0], 0) - 10
x_hi = max(df.enu_x.max(), goal[0], 0) + 10
y_lo = min(df.enu_y.min(), goal[1], 0) - 5
y_hi = max(df.enu_y.max(), goal[1], 0) + 5
ax.set_xlim(x_lo, x_hi)
ax.set_ylim(y_lo, y_hi)
ax.set_xlabel('ENU east x (m)')
ax.set_ylabel('ENU north y (m)')
ax.set_title('Top-Down (x vs y)')
ax.grid(alpha=0.25)
mode_leg = [
    Patch(facecolor=mode_colors['CA'], label='CA mode'),
    Patch(facecolor=mode_colors['PF'], label='PF mode'),
    Line2D([0],[0], marker='o', color='w', markerfacecolor='#2060ff', markersize=10, label='start', markeredgecolor='k'),
    Line2D([0],[0], marker='*', color='w', markerfacecolor='#ffd900', markersize=14, label='goal', markeredgecolor='k'),
]
ax.legend(handles=mode_leg, loc='best', fontsize=8, framealpha=0.9, ncol=2)

# (0,1) Side view
ax = axes[0, 1]
colored_line(ax, df.enu_x.values, df.alt.values, lw=2.2)
ax.scatter([0], [df.alt.iloc[0]], s=200, c='#2060ff', marker='o', edgecolors='k', lw=1.2, zorder=5)
ax.scatter([goal[0]], [goal[2]], s=280, c='#ffd900', marker='*', edgecolors='k', lw=1.2, zorder=5)
ax.axhline(goal[2], color='gray', ls='--', alpha=0.5, lw=1)
ax.set_xlim(x_lo, x_hi)
alt_lo = min(df.alt.min(), goal[2], 0) - 3
ax.set_ylim(alt_lo, max(df.alt.max(), goal[2]) + 3)
ax.set_xlabel('ENU east x (m)')
ax.set_ylabel('altitude z (m)')
ax.set_title('Side view (x vs z)')
ax.grid(alpha=0.25)

# Time-series panels (sharex)
t_min, t_max = df.t.min(), df.t.max()

# (1,0) x over time
ax = axes[1, 0]; shade_bg(ax)
colored_line(ax, df.t.values, df.enu_x.values)
ax.axhline(goal[0], color='gray', ls='--', alpha=0.6, label=f'goal x={goal[0]:.0f}')
ax.set_xlim(t_min, t_max)
ax.set_ylabel('ENU east x (m)')
ax.set_title('ENU x over time')
ax.grid(alpha=0.25)
ax.legend(loc='best', fontsize=9)

# (1,1) y over time
ax = axes[1, 1]; shade_bg(ax)
colored_line(ax, df.t.values, df.enu_y.values)
ax.axhline(0, color='gray', ls='--', alpha=0.5)
ax.set_xlim(t_min, t_max)
ax.set_ylim(-y_abs_max - 1, y_abs_max + 1)
ax.set_ylabel('ENU north y (m)')
ax.set_title('ENU y over time')
ax.grid(alpha=0.25)

# (2,0) z over time
ax = axes[2, 0]; shade_bg(ax)
colored_line(ax, df.t.values, df.alt.values)
ax.axhline(goal[2], color='gray', ls='--', alpha=0.6, label=f'goal z={goal[2]:.0f}')
ax.set_xlim(t_min, t_max)
ax.set_ylim(0, max(df.alt.max(), goal[2]) + 3)
ax.set_ylabel('altitude z (m)')
ax.set_title('ENU z (altitude) over time')
ax.grid(alpha=0.25)
ax.legend(loc='best', fontsize=9)

# (2,1) velocity time series
ax = axes[2, 1]; shade_bg(ax)
ax.plot(df.t, df.enu_vx, color='r', lw=1, label='vx (east)', zorder=3)
ax.plot(df.t, df.enu_vy, color='g', lw=1, label='vy (north)', zorder=3)
ax.plot(df.t, df.enu_vz, color='b', lw=1, label='vz (up)', zorder=3)
ax.axhline(0, color='k', ls='-', alpha=0.3, lw=0.5)
ax.set_xlim(t_min, t_max)
ax.set_ylabel('velocity (m/s)')
ax.set_title('ENU velocity over time')
ax.grid(alpha=0.25)
ax.legend(loc='best', fontsize=9, ncol=3)

# (3,0) fusion
ax = axes[3, 0]; shade_bg(ax)
ax.fill_between(df.t, 0, df.fusion_w, color='#3ab363', alpha=0.5, zorder=3)
ax.plot(df.t, df.fusion_w, color='#2a6a3a', lw=1.5, zorder=4)
ax.axhline(0.5, color='gray', ls=':', alpha=0.5, label='pf_cap 0.5')
ax.set_xlim(t_min, t_max)
ax.set_ylim(-0.05, 0.6)
ax.set_xlabel('time (s)')
ax.set_ylabel('fusion_weight')
ax.set_title('Fusion weight')
ax.grid(alpha=0.25)
ax.legend(loc='best', fontsize=9)

# (3,1) lidar nearest dist + min annotation
ax = axes[3, 1]; shade_bg(ax)
if 'lidar_3d_dist' in df.columns:
    ax.plot(df.t, df.lidar_3d_dist, color='black', lw=1.3, label='3D nearest', zorder=3)
    min_idx = df.lidar_3d_dist.idxmin() if df.lidar_3d_dist.notna().any() else None
    if min_idx is not None:
        min_t = df.t.loc[min_idx]; min_d = df.lidar_3d_dist.loc[min_idx]
        ax.scatter([min_t], [min_d], s=120, c='red', marker='v', edgecolors='k',
                   zorder=6, label=f'min {min_d:.2f}m @ t={min_t:.0f}s')
        ax.axhline(min_d, color='red', ls=':', alpha=0.4)
ax.axhline(4.0, color='#3ab363', ls=':', alpha=0.5, label='dist_pf 4m')
ax.axhline(3.0, color='#e54f4f', ls=':', alpha=0.5, label='dist_ca 3m')
ax.set_xlim(t_min, t_max)
ax.set_ylim(0, 25)
ax.set_xlabel('time (s)')
ax.set_ylabel('lidar nearest dist (m)')
ax.set_title('Lidar nearest obstacle distance')
ax.grid(alpha=0.25)
ax.legend(loc='best', fontsize=8, ncol=2)

# title
fin = df.iloc[-1]
title = (
    f'Integration flight  |  goal ({goal[0]:.0f}, {goal[1]:.0f}, {goal[2]:.0f})  '
    f'final ({fin.enu_x:.1f}, {fin.enu_y:.1f}, {fin.alt:.1f})  err ({err_x:.1f}, {err_y:.1f}, {err_z:.1f})m\n'
    f'duration {tot:.0f}s  |  CA {ca_dur:.1f}s ({ca_dur/tot*100:.0f}%)  PF {pf_dur:.1f}s ({pf_dur/tot*100:.0f}%)  '
    f'switches {sum(1 for i in range(1,len(bands)) if bands[i][2]!=bands[i-1][2])}'
)
fig.suptitle(title, fontsize=13, y=0.998)

plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(out_png, dpi=120, bbox_inches='tight')
print(f'saved: {out_png}')
print(f'CA {ca_dur:.1f}s ({ca_dur/tot*100:.0f}%) PF {pf_dur:.1f}s ({pf_dur/tot*100:.0f}%)')
if 'lidar_3d_dist' in df.columns and df.lidar_3d_dist.notna().any():
    print(f'min lidar dist: {df.lidar_3d_dist.min():.2f}m')
