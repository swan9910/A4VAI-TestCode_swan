#!/bin/bash
# PP (PSO) + PF + CA full integration
#
# 흐름:
#   1) pp_only_integration.sh 실행 — user 가 GPS 입력 → path_final_px4.txt 생성
#   2) path_final_px4.txt 읽고 alt +5 offset 적용해서 wp.csv 직접 작성
#   3) 마지막 wp 로 GOAL 계산
#   4) launch_integration_cpp.sh 를 SKIP_WP_GEN=1 + GOAL/TAKEOFF env 로 실행
#      (wp.csv 안 덮어쓰고 우리가 쓴 거 그대로 사용)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PP_SCRIPT="${SCRIPT_DIR}/../path_planning_unit_test/pp_only_integration.sh"
INT_SCRIPT="${SCRIPT_DIR}/launch_integration_cpp.sh"
PX4_TXT="/home/user/a4vai_ws/pathplanning/pathplanning/Results_Images/path_final_px4.txt"
INPUT_GPS="/home/user/a4vai_ws/pathplanning/pathplanning/Results_Images/input_gps.txt"
WP_CSV="/home/user/a4vai_ws/install/algorithm_test/lib/python3.12/site-packages/algorithm_test/path_following_unit_test/wp.csv"
ALT_OFFSET=${ALT_OFFSET:-7}
ARRIVAL_THRESHOLD=${ARRIVAL_THRESHOLD:-5.0}

echo "╔══════════════════════════════════════════════════════════╗"
echo "║         PP (PSO) + PF + CA Full Integration              ║"
echo "╚══════════════════════════════════════════════════════════╝"

# 1) PSO 경로 계획 (interactive — user GPS 입력)
echo ""
echo "[Step 1/3] PSO 경로 계획 실행 (GPS 입력 필요)"
bash "$PP_SCRIPT"

# 2) path_final_px4.txt → wp.csv (alt +ALT_OFFSET)
echo ""
echo "[Step 2/3] path_final_px4.txt → wp.csv 변환 (alt +${ALT_OFFSET}m)"
if [ ! -f "$PX4_TXT" ]; then
    echo "  ✗ $PX4_TXT 없음. PSO 가 실패했거나 결과 파일 안 생성됨." >&2
    exit 1
fi

read -r GOAL_X GOAL_Y GOAL_Z N_WPS <<< $(python3 - <<PYEOF
import numpy as np
d = np.loadtxt('$PX4_TXT')
# 모든 wp 에 takeoff_alt 더함 (지면 위 ALT_OFFSET m clearance)
d[:, 2] += $ALT_OFFSET
# 첫 두 wp 만 z = takeoff_alt 로 강제 (이륙 hover 위치 일치)
if len(d) >= 1:
    d[0, 2] = $ALT_OFFSET
if len(d) >= 2:
    d[1, 2] = $ALT_OFFSET
with open('$WP_CSV', 'w') as f:
    f.write('x,y,z\n')
    for r in d:
        f.write(f'{r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f}\n')
# PF NED → ENU: GOAL_X=east=col2, GOAL_Y=north=col1, GOAL_Z=alt=col3
print(f'{d[-1,1]:.4f} {d[-1,0]:.4f} {d[-1,2]:.4f} {len(d)}')
PYEOF
)

TAKEOFF_ALT=$(printf '%.1f' "$ALT_OFFSET")  # DOUBLE 강제 (ROS2 param type)
echo "  wp.csv 작성 완료: ${N_WPS} wps"
echo "  Goal ENU: (east=${GOAL_X}, north=${GOAL_Y}, alt=${GOAL_Z})"
echo "  Takeoff alt: ${TAKEOFF_ALT}m"

# 2.5) GPS 도달 monitor (background — stdout 으로 도달 시 출력)
if [ -f "$INPUT_GPS" ]; then
    echo ""
    echo "[Step 2.5/3] wp_arrival_monitor 시작 (threshold=${ARRIVAL_THRESHOLD}m)"
    mkdir -p /home/user/a4vai_ws/logs
    # ros2 환경 source (wrapper 단독 실행 시 px4_msgs 못 찾을 수 있음)
    source /opt/ros/jazzy/setup.bash
    source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash 2>/dev/null || true
    nohup python3 "${SCRIPT_DIR}/wp_arrival_monitor.py" \
        --input-gps "$INPUT_GPS" --threshold "$ARRIVAL_THRESHOLD" \
        > /home/user/a4vai_ws/logs/wp_arrival.log 2>&1 &
    MON_PID=$!
    echo "  monitor PID=$MON_PID  log=logs/wp_arrival.log (int_script rm 회피)"
else
    echo "  ⚠ $INPUT_GPS 없음 — monitor skip"
fi

# 3) 통합 launcher 실행 (wp.csv 안 덮어쓰게 SKIP_WP_GEN, GOAL/TAKEOFF env override)
echo ""
echo "[Step 3/3] CA + PF 통합 비행 launcher 실행"
export SKIP_WP_GEN=1
export GOAL_X GOAL_Y GOAL_Z TAKEOFF_ALT
finalize_overlay() {
    echo ""
    echo "[overlay] PSO + actual flight 합성 이미지 생성"
    python3 ${SCRIPT_DIR}/../lib/plot_overlay.py 2>&1 | tail -2
}
trap finalize_overlay EXIT INT TERM

bash "$INT_SCRIPT"

