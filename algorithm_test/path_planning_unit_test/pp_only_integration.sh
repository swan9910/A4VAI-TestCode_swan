#!/bin/bash
# pp_only_integration.sh
# Docker 내부에서 실행:
#   bash /home/user/a4vai_ws/pathplanning/pp_only_integration.sh

set -e

# ─── ROS2 환경 ─────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source /home/user/a4vai_ws/install/setup.bash 2>/dev/null || true
export PYTHONPATH=/usr/local/lib/python3.12/dist-packages:${PYTHONPATH}

# Plan2WP 의 hardcoded workspace 경로 → 우리 container 경로로 symlink (sim 재기동 시마다 필요)
mkdir -p /home/user/workspace/ros2/ros2_ws/src 2>/dev/null
ln -sfn /home/user/a4vai_ws/pathplanning /home/user/workspace/ros2/ros2_ws/src/pathplanning 2>/dev/null || true

# ─── 경로 설정 (Docker 내부) ────────────────────────────────────────
CONVERT_PY="/home/user/a4vai_ws/algorithm_test/algorithm_test/path_planning_unit_test/convert_waypoints.py"
HEIGHTMAP="/home/user/a4vai_ws/pathplanning/pathplanning/map/IMG_0268_1000.png"
RESULTS="/home/user/a4vai_ws/pathplanning/pathplanning/Results_Images"
WAIT_TIMEOUT=600   # PSO 최대 대기 시간 (초)
HEARTBEAT_WAIT=5   # Plan2WP 시작 후 heartbeat 대기 (초)

# ───────────────────────────────────────────────────────────────────

# GPS → (row col z),  z = heightmap 픽셀값
gps2pix() {
    local lat=$1 lon=$2
    local result row col z
    result=$(python3 "$CONVERT_PY" --gps "$lat" "$lon")
    row=$(echo "$result" | grep -oP 'row=\K-?\d+')
    col=$(echo "$result" | grep -oP 'col=\K-?\d+')
    z=$(python3 -c "import cv2; img=cv2.imread('$HEIGHTMAP',0); print(int(img[$row,$col]))")
    echo "$row $col $z"
}


# heartbeat 퍼블리셔 시작 (background, 1초 주기)
start_heartbeats() {
    for topic in /controller_heartbeat /path_following_heartbeat /collision_avoidance_heartbeat; do
        ros2 topic pub "$topic" std_msgs/msg/Bool "{data: true}" --rate 1 \
            > /dev/null 2>&1 &
    done
    echo "  heartbeat 퍼블리시 시작 (3개 토픽)"
}

# heartbeat 퍼블리셔 전체 종료
stop_heartbeats() {
    pkill -f "std_msgs/msg/Bool" 2>/dev/null || true
}

# Plan2WP 재시작 (background, 이전 인스턴스 kill 후)
restart_plan2wp() {
    pkill -f "Plan2WP" 2>/dev/null || true
    sleep 1
    ros2 run pathplanning Plan2WP > "$RESULTS/plan2wp_last.log" 2>&1 &
    PP_PID=$!
    echo "  Plan2WP 시작 (PID=$PP_PID), heartbeat 대기 ${HEARTBEAT_WAIT}초..."
    sleep $HEARTBEAT_WAIT
}

# /global_waypoint_setpoint 무한 퍼블리시 (1Hz, background — wait_plan 끝나면 kill)
pub_segment() {
    local sc=$1 sr=$2 sz=$3 gc=$4 gr=$5 gz=$6
    echo "  publish: start=[col=$sc, row=$sr, z=$sz] → goal=[col=$gc, row=$gr, z=$gz]"
    ros2 topic pub --rate 1 /global_waypoint_setpoint \
        custom_msgs/msg/GlobalWaypointSetpoint \
        "{start_point: [${sc}.0, ${sr}.0, ${sz}.0], goal_point: [${gc}.0, ${gr}.0, ${gz}.0]}" \
        > /dev/null 2>&1 &
    PUB_PID=$!
}

# waypoint.txt 갱신 감지로 PSO 완료 대기
wait_plan() {
    local before elapsed=0
    before=$(stat -c %Y "$RESULTS/waypoint.txt" 2>/dev/null || echo 0)
    echo -n "  PSO 대기"
    while true; do
        sleep 3
        elapsed=$((elapsed + 3))
        local after
        after=$(stat -c %Y "$RESULTS/waypoint.txt" 2>/dev/null || echo 0)
        if [ "$after" -gt "$before" ]; then
            echo "  완료 (${elapsed}초)"
            kill $PUB_PID 2>/dev/null || true   # 무한 publisher 정리
            return 0
        fi
        if [ "$elapsed" -ge "$WAIT_TIMEOUT" ]; then
            echo "  타임아웃 (${WAIT_TIMEOUT}초 초과)"
            kill $PUB_PID $PP_PID 2>/dev/null || true
            return 1
        fi
        echo -n "."
    done
}

# ─── 메인 ──────────────────────────────────────────────────────────
echo ""
echo "╔═══════════════════════════════════════════╗"
echo "║      PP Only Integration Script          ║"
echo "╚═══════════════════════════════════════════╝"
echo ""

# 경유점 수 입력
echo -n "중간 경유점 수 (0=직항, 1=wp1개, ...): "
read -r N_WP
N_SEGS=$((N_WP + 1))
N_PTS=$((N_WP + 2))
echo "  → PSO $N_SEGS 회 실행 예정"
echo ""

# GPS 좌표 입력
declare -a ROWS COLS ZS LABELS LATS LONS
for ((i=0; i<N_PTS; i++)); do
    if   [ $i -eq 0 ];            then LABELS[$i]="Start"
    elif [ $i -eq $((N_PTS-1)) ]; then LABELS[$i]="Goal"
    else                               LABELS[$i]="WP$i"
    fi
done

echo "[ GPS 좌표 입력 ]  형식: lat,lon  또는  lat lon  (예: 36.729077, 127.441927)"
echo "  $N_PTS 점을 한 줄씩 입력 (한꺼번에 paste 가능). 잘못 입력하면 다시 받음."

for ((i=0; i<N_PTS; i++)); do
    while true; do
        echo -n "  ${LABELS[$i]}: "
        read -r input
        input=$(echo "$input" | tr -d '(),')
        lat=$(echo $input | awk '{print $1}')
        lon=$(echo $input | awk '{print $2}')

        # 유효성 검사: 숫자 2개인지, 위경도 범위인지
        if ! python3 -c "
lat, lon = float('$lat'), float('$lon')
assert -90 <= lat <= 90, 'lat out of range'
assert -180 <= lon <= 180, 'lon out of range'
" 2>/dev/null; then
            echo "    ✗ 올바르지 않은 입력입니다. 다시 입력해주세요. (예: 36.729077 127.441927)"
            continue
        fi

        read -r r c z <<< "$(gps2pix "$lat" "$lon")"

        # 픽셀이 맵 범위 내인지 확인 (0~999)
        if [ "$r" -lt 0 ] || [ "$r" -gt 999 ] || [ "$c" -lt 0 ] || [ "$c" -gt 999 ]; then
            echo "    ✗ 픽셀 좌표(row=$r, col=$c)가 맵 범위(0~999)를 벗어납니다. 다시 입력해주세요."
            continue
        fi

        ROWS[$i]=$r; COLS[$i]=$c; ZS[$i]=$z
        LATS[$i]=$lat; LONS[$i]=$lon
        echo "    → pixel(row=$r, col=$c, z=$z)"
        break
    done
done

# 사용자 입력 GPS 저장 (도달 monitor 가 읽음)
INPUT_GPS_FILE="$RESULTS/input_gps.txt"
> "$INPUT_GPS_FILE"
for ((i=0; i<N_PTS; i++)); do
    echo "${LABELS[$i]} ${LATS[$i]} ${LONS[$i]}" >> "$INPUT_GPS_FILE"
done
echo "  입력 GPS 저장: $INPUT_GPS_FILE ($N_PTS 점)"

echo ""
echo "[ heartbeat 퍼블리시 ]"
start_heartbeats

echo ""
echo "[ PSO 경로계획 시작 ]"
declare -a SEG_FILES

for ((seg=0; seg<N_SEGS; seg++)); do
    i=$seg
    j=$((seg+1))
    echo ""
    echo "── Seg $((seg+1))/$N_SEGS: ${LABELS[$i]} → ${LABELS[$j]} ──────────────"

    restart_plan2wp
    pub_segment "${COLS[$i]}" "${ROWS[$i]}" "${ZS[$i]}" "${COLS[$j]}" "${ROWS[$j]}" "${ZS[$j]}"
    wait_plan

    seg_file="$RESULTS/seg_${seg}.txt"
    cp "$RESULTS/waypoint.txt" "$seg_file"
    SEG_FILES[$seg]="$seg_file"
    echo "  → seg_${seg}.txt 저장"

    kill $PP_PID 2>/dev/null || true
    sleep 1
done

# 구간 파일 합치기 (경계점 중복 제거)
echo ""
echo "[ 최종 경로 합치기 ]"
FINAL="$RESULTS/path_final.txt"
> "$FINAL"
for ((seg=0; seg<N_SEGS; seg++)); do
    if [ $seg -eq 0 ]; then
        cat "${SEG_FILES[$seg]}" >> "$FINAL"
    else
        tail -n +2 "${SEG_FILES[$seg]}" >> "$FINAL"
    fi
done

TOTAL_WP=$(wc -l < "$FINAL")
echo "  → path_final.txt (총 $TOTAL_WP 웨이포인트)"

# Gazebo 좌표 변환
FINAL_PX4="$RESULTS/path_final_px4.txt"
python3 "$CONVERT_PY" "$FINAL" "$FINAL_PX4"
echo "  → path_final_px4.txt 저장"

stop_heartbeats

echo ""
echo "[ 결과 이미지 저장 ]"
python3 - <<PYEOF
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import cv2
import numpy as np

img  = cv2.imread('$HEIGHTMAP', cv2.IMREAD_GRAYSCALE)
data = np.loadtxt('$FINAL')

plt.figure(figsize=(10, 10))
plt.imshow(img, cmap='gray')
plt.plot(data[:,1], data[:,0], 'r-', linewidth=1.5)
plt.plot(data[0,1],  data[0,0],  'go', markersize=10, label='Start')
plt.plot(data[-1,1], data[-1,0], 'bo', markersize=10, label='Goal')
plt.legend()
plt.title('2D Path on Heightmap')
plt.xlabel('X')
plt.ylabel('Y')
out = '$RESULTS/path_final_2d.png'
plt.savefig(out, dpi=150, bbox_inches='tight')
plt.close()
print(f'  → path_final_2d.png 저장')
PYEOF

echo ""
echo "╔═══════════════════════════════════════════╗"
echo "║  완료!  Results_Images/path_final.txt        (픽셀)  ║"
echo "║          Results_Images/path_final_px4.txt (PX4) ║"
echo "║          Results_Images/path_final_2d.png             ║"
echo "╚═══════════════════════════════════════════╝"

# ─── PSO 결과 UDP 전송 (실시간 viz 컴퓨터로) ───────────────
if [ -f "$RESULTS/path_final.txt" ]; then
  python3 /home/user/a4vai_ws/algorithm_test/algorithm_test/lib/pp_path_broadcaster.py \
    --ip 100.68.0.70 --port 45680 \
    --path "$RESULTS/path_final.txt" || true
fi
