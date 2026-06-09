#!/usr/bin/env python3
"""자세 명령 vs 실제 자세 시계열 plot.
   actual: roll_deg/pitch_deg/yaw_deg from drone odom (ENU)
   pf_cmd: PF 가 발행한 quat → euler (NED body)
   att_sp: PX4 vehicle_attitude_setpoint → euler (NED body)
"""
import sys, math
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

csv_path = sys.argv[1]
out_png  = sys.argv[2]

df = pd.read_csv(csv_path)
df['alt'] = -df.ned_z
df = df[df.alt > 0.1].reset_index(drop=True)

def quat_to_euler_deg(qw, qx, qy, qz):
    sinr = 2*(qw*qx + qy*qz)
    cosr = 1 - 2*(qx*qx + qy*qy)
    roll = np.arctan2(sinr, cosr)
    sinp = np.clip(2*(qw*qy - qz*qx), -1, 1)
    pitch = np.arcsin(sinp)
    siny = 2*(qw*qz + qx*qy)
    cosy = 1 - 2*(qy*qy + qz*qz)
    yaw = np.arctan2(siny, cosy)
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

pf_r,  pf_p,  pf_y  = quat_to_euler_deg(df.pf_qw, df.pf_qx, df.pf_qy, df.pf_qz)
sm_r,  sm_p,  sm_y  = quat_to_euler_deg(df.pf_cmd_qw, df.pf_cmd_qx, df.pf_cmd_qy, df.pf_cmd_qz)

# mode background shading
def make_bands(fw_series, t_series):
    bands = []
    if len(fw_series) == 0: return bands
    cur_mode = 'PF' if fw_series.iloc[0] >= 0.5 else 'CA'
    ss = t_series.iloc[0]
    for i, w in enumerate(fw_series):
        if np.isnan(w): continue
        m = 'PF' if w >= 0.5 else 'CA'
        if m != cur_mode:
            bands.append((ss, t_series.iloc[i], cur_mode))
            cur_mode = m; ss = t_series.iloc[i]
    bands.append((ss, t_series.iloc[-1], cur_mode))
    return bands

bands = make_bands(df.fusion_w, df.t)
mode_colors = {'CA': '#ffe5e5', 'PF': '#e5f5e5'}

def shade(ax):
    for s, e, m in bands:
        ax.axvspan(s, e, color=mode_colors[m], alpha=0.5, zorder=0)

fig, axes = plt.subplots(3, 1, figsize=(15, 11), sharex=True)
labels = ['roll', 'pitch', 'yaw']
for i, (actual, pf, sm, lbl) in enumerate([
        (df.roll_deg,  pf_r, sm_r, 'roll'),
        (df.pitch_deg, pf_p, sm_p, 'pitch'),
        (df.yaw_deg,   pf_y, sm_y, 'yaw'),
]):
    ax = axes[i]
    shade(ax)
    ax.plot(df.t, actual, color='k',       lw=1.5, label='actual (drone)', zorder=4)
    ax.plot(df.t, pf,     color='#e54f4f', lw=1.0, alpha=0.7, label='PF raw cmd', zorder=3)
    ax.plot(df.t, sm,     color='#2060ff', lw=1.0, alpha=0.8, ls='--', label='PF smoothed cmd (→PX4)', zorder=3)
    ax.set_ylabel(f'{lbl} (deg)')
    ax.grid(alpha=0.25)
    if i == 0:
        ax.legend(loc='upper left', fontsize=9, framealpha=0.9)
axes[-1].set_xlabel('time (s)')
fig.suptitle('Attitude commands vs actual (CA = red bg, PF = green bg)', fontsize=13)
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(out_png, dpi=120, bbox_inches='tight')
print(f'saved: {out_png}')
