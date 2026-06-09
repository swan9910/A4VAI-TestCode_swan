"""
PSO 웨이포인트 픽셀 좌표 -> Gazebo 절대 좌표 변환 스크립트

변환식 기준점 (affine, 3점):
  - 원통 건물: pixel(row=159, col=515) -> Gazebo(-149.09, -132.67)
  - 사각 건물: pixel(row=287, col=731) -> Gazebo(-12.0,   -31.81)
  - 후보B 건물: pixel(row=455, col=565) -> Gazebo(-125.0,   110.81)

좌표계:
  - path_x = row, path_y = col
  - gazebo_x = A*row + B*col + C
  - gazebo_y = D*row + E*col + F

사용법:
  python3 convert_waypoints.py waypoint1.txt
  python3 convert_waypoints.py waypoint1.txt waypoint1_gazebo.txt

GPS -> pixel 변환:
  python3 convert_waypoints.py --gps <lat> <lon>
  예시: python3 convert_waypoints.py --gps 36.729247 127.441992
"""

import numpy as np
import sys
import os

# ─── affine 변환 파라미터 ────────────────────────────────────────
# gazebo_x = A*row + B*col + C
A, B, C = -0.0287, 0.6517, -480.1430
# gazebo_y = D*row + E*col + F
D, E, F =  0.8264, -0.0228, -252.3366
# pixel_z -> gazebo_z
SZ, OZ = 0.1469, 0.9644

# ─── GPS -> pixel 직접 affine 파라미터 (3점 피팅) ───────────────
# 기준점:
#   원통 건물: GPS(36.729247, 127.441992) -> pixel(159, 515)
#   사각 건물: GPS(36.728312, 127.443570) -> pixel(287, 731)
#   후보B 건물: GPS(36.727069, 127.442312) -> pixel(455, 565)
# row = _Ga*lat + _Gb*lon + _Gc
_Ga, _Gb, _Gc = -135809.724625,  645.061771,  4906139.963628
# col = _Gd*lat + _Ge*lon + _Gf
_Gd, _Ge, _Gf =   -3116.948679, 135035.268052, -17094165.372942
# ────────────────────────────────────────────────────────────────

RESULTS_DIR = "/home/user/workspace/ros2/ros2_ws/src/pathplanning/pathplanning/Results_Images/"


def gps_to_pixel(lat, lon):
    """GPS(lat, lon) -> pixel(row, col) 변환 (3점 affine, 반올림 정수)"""
    row = _Ga * lat + _Gb * lon + _Gc
    col = _Gd * lat + _Ge * lon + _Gf
    return int(round(row)), int(round(col))


def convert(input_path, output_path):
    data = np.loadtxt(input_path)

    def to_gazebo(row, col, z):
        gx = A * row + B * col + C
        gy = D * row + E * col + F
        gz = SZ * z + OZ
        return gx, gy, gz

    start_gx, start_gy, start_gz = to_gazebo(data[0, 0], data[0, 1], data[0, 2])

    result = []
    for row, col, z in data:
        gx, gy, gz = to_gazebo(row, col, z)
        dx = gx - start_gx
        dy = gy - start_gy
        dz = gz - start_gz
        result.append([-dy, dx, dz])

    result = np.array(result)
    np.savetxt(output_path, result, fmt="%.4f", header="x y z (local relative to start, -y x z = PX4 NED-like)")

    print(f"변환 완료: {output_path}")
    print(f"웨이포인트 수: {len(result)}")
    for i, (x, y, z) in enumerate(result):
        label = "Start" if i == 0 else ("Goal" if i == len(result)-1 else f"WP{i}")
        print(f"  {label}: ({x:.2f}, {y:.2f}, {z:.2f})")

    return result


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "--gps":
        lat, lon = float(sys.argv[2]), float(sys.argv[3])
        row, col = gps_to_pixel(lat, lon)
        print(f"GPS ({lat}, {lon}) -> pixel (row={row}, col={col})")
        sys.exit(0)

    if len(sys.argv) < 2:
        print("사용법: python3 convert_waypoints.py <input.txt> [output.txt]")
        print("       python3 convert_waypoints.py --gps <lat> <lon>")
        print(f"예시:   python3 convert_waypoints.py {RESULTS_DIR}waypoint1.txt")
        sys.exit(1)

    input_path = sys.argv[1]
    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        base = os.path.splitext(os.path.basename(input_path))[0]
        output_path = os.path.join(os.path.dirname(input_path), base + "_px4.txt")

    convert(input_path, output_path)
