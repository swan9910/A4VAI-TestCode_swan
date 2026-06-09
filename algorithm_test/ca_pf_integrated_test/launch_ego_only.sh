#!/bin/bash
# ego-only 단위 테스트 (path following / mode_switcher 없음)
# 전제: PX4 SITL + Gazebo (RealGazebo) 가 이미 떠 있음
#
# 흐름:
#   1) flight_logger          : 모든 topic 50Hz CSV 기록 (offboard 와 동시)
#   2) offboard.py            : PX4 odom → /ego_odom (ENU), takeoff
#   3) correct_pc_transformer : /lidar/points → /lidar/points_world
#   4) airsim_px4.launch      : ego_planner_node + traj_server
#   5) ego goal --once 발행   : ego-planner trajectory 생성
#
# 단위 테스트 의도:
#   - fusion_weight 발행 안 함 → PX4 패치는 ego output 만 사용
#   - path_following_att_cmd 발행 안 함 → SLERP 비활성
#   - ego-planner 단독으로 c-track 장애물 회피 확인
#
# Logs: ${LOG_DIR}/  (영구 보존)
# CSV:  /home/user/flight_logs/<timestamp>_ego_only.csv

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LOG_DIR=/home/user/ego_only_logs
mkdir -p ${LOG_DIR}
rm -f ${LOG_DIR}/*.log

source /opt/ros/jazzy/setup.bash
source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash 2>/dev/null || true
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true
source /home/user/a4vai_ws/install/setup.bash 2>/dev/null || true

# CUDA toolkit (이 테스트엔 직접 필요 없지만 통일)
export PATH=/usr/local/cuda-12.8/bin:${PATH}
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH}

TAKEOFF_ALT=3.0
GOAL_X=70.0
GOAL_Y=0.0
GOAL_Z=3.0
DURATION=120        # 70m / 1m/s ≈ 70s + takeoff/여유

echo "[0/5] flight_logger (offboard 와 동시 시작 — takeoff 단계 데이터까지 잡음)"
mkdir -p /home/user/flight_logs
DIAG_CSV=/home/user/flight_logs/$(date +%Y%m%d_%H%M%S)_ego_only.csv
python3 ${SCRIPT_DIR}/flight_logger.py \
  --duration ${DURATION} --csv ${DIAG_CSV} \
  > ${LOG_DIR}/flight_logger.log 2>&1 &
echo "  pid=$!  csv=${DIAG_CSV}"

sleep 1

echo "[1/5] offboard.py (takeoff alt=${TAKEOFF_ALT}m)"
python3 /home/user/ros2_ws/src/ego-planner-a4vai/offboard.py \
  --ros-args -p takeoff_alt:=${TAKEOFF_ALT} -p yaw_offset_deg:=0.0 \
  > ${LOG_DIR}/offboard.log 2>&1 &
echo "  pid=$!"

sleep 1

echo "[2/5] correct_pc_transformer"
python3 /home/user/ros2_ws/src/ego-planner-a4vai/correct_pc_transformer.py \
  > ${LOG_DIR}/pc_transformer.log 2>&1 &
echo "  pid=$!"

sleep 1

echo "[3/5] ego_planner (airsim_px4 — defaults 그대로)"
ros2 launch ego_planner airsim_px4.launch.py \
  > ${LOG_DIR}/ego_planner.log 2>&1 &
echo "  pid=$!"

# offboard 의 takeoff 가 완료될 시간 확보
sleep 7

echo ""
echo "[4/5] publishing ego goal × 3 (subscriber race 회피): (${GOAL_X}, ${GOAL_Y}, ${GOAL_Z})"
for i in 1 2 3; do
  ros2 topic pub --once /move_base_simple/goal geometry_msgs/PoseStamped \
    "{header: {frame_id: 'world'}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}, orientation: {w: 1.0}}}" \
    > ${LOG_DIR}/goal_pub.log 2>&1
  echo "  goal pub $i done"
  sleep 1
done

# ego_planner 가 실제로 받았는지 확인 (EXEC_TRAJ 전환 대기)
echo "[ ... waiting for ego EXEC_TRAJ ... ]"
until grep -q 'EXEC_TRAJ' ${LOG_DIR}/ego_planner.log 2>/dev/null; do sleep 1; done
echo "  ego trajectory active"

echo ""
echo "=== ego-only test running. ==="
echo "  Logs: ${LOG_DIR}/"
echo "  CSV:  ${DIAG_CSV}"
echo ""
echo "분석: python3 ${SCRIPT_DIR}/plot_pretty4.py ${DIAG_CSV}"
echo "Stop: pkill -f 'offboard.py|correct_pc_transformer|airsim_px4|flight_logger.py|ros2 launch ego_planner'"

# flight_logger 종료까지 대기 (DURATION 후 자동 종료)
wait
