#!/bin/bash
# path-following 단독 단위 테스트 (ego-planner 없음, mode_switcher 없음)
# 전제: PX4 SITL + Gazebo (RealGazebo) 가 이미 떠 있음
#
# 흐름:
#   1) flight_logger              : 모든 topic 50Hz CSV 기록 (offboard 와 동시 시작)
#   2) offboard.py                : PX4 odom → /ego_odom (ENU), takeoff
#   3) correct_pc_transformer     : /lidar/points → /lidar/points_world (logger 만 사용)
#   4) node_att_ctrl              : path following 자세 제어
#   5) node_MPPI_output           : MPPI output → /pf_att_2_control
#   6) path_following_bridge_test : orchestrator
#       - /pf_att_2_control → /vehicle1/fmu/in/path_following_att_cmd
#       - fusion_weight 0 → 1.0 5초 ramp (자체 publish)
#       - takeoff 후 wp.csv 의 waypoint 단발 발행 → node_att_ctrl 받아서 따라감
#
# 단위 테스트 의도:
#   - ego-planner / traj_server / mode_switcher 없음
#   - SLERP 가 path following 자세를 그대로 PX4 에 전달하는지 확인
#   - wp.csv = (0,0,3) → (0,0,3) → (-16.5, 70, 3) NED
#       즉, ENU (0,0,3) → ENU (70, -16.5, 3) (동쪽 70m + 남쪽 16.5m, 장애물 우회 경로)
#
# Logs: ${LOG_DIR}/  (영구 보존)
# CSV:  /home/user/flight_logs/<timestamp>_pf_only.csv

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LOG_DIR=/home/user/pf_only_logs
mkdir -p ${LOG_DIR}
rm -f ${LOG_DIR}/*.log

source /opt/ros/jazzy/setup.bash
source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash 2>/dev/null || true
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true
source /home/user/a4vai_ws/install/setup.bash 2>/dev/null || true

# CUDA toolkit (nvcc 필요 - MPPI pycuda 컴파일)
export PATH=/usr/local/cuda-12.8/bin:${PATH}
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH}

TAKEOFF_ALT=3.0
DURATION=120

# pf-only 모드: wp.csv 를 ENU (70, -16.5, 3) 우회 path 로 설정 (장애물 회피 기동).
# integration-bridge launcher 와 wp.csv 공유하므로 자기 의도에 맞게 명시적으로 작성.
WP_CSV=/home/user/a4vai_ws/install/algorithm_test/lib/python3.12/site-packages/algorithm_test/path_following_unit_test/wp.csv
cat > ${WP_CSV} <<EOF
x,y,z
0.0, 0.0, 3.0
0.0, 0.0, 3.0
-16.5, 70.0, 3.0
EOF
echo "wp.csv set to NED (-16.5, 70, 3) = ENU (70, -16.5, 3) for pf-only mode"

echo "[0/6] flight_logger (offboard 와 동시 시작 — takeoff 단계 데이터까지 잡음)"
mkdir -p /home/user/flight_logs
DIAG_CSV=/home/user/flight_logs/$(date +%Y%m%d_%H%M%S)_pf_only.csv
python3 ${SCRIPT_DIR}/flight_logger.py \
  --duration ${DURATION} --csv ${DIAG_CSV} \
  > ${LOG_DIR}/flight_logger.log 2>&1 &
echo "  pid=$!  csv=${DIAG_CSV}"

sleep 1

echo "[1/6] offboard.py (takeoff alt=${TAKEOFF_ALT}m)"
python3 /home/user/ros2_ws/src/ego-planner-a4vai/offboard.py \
  --ros-args -p takeoff_alt:=${TAKEOFF_ALT} -p yaw_offset_deg:=0.0 \
  > ${LOG_DIR}/offboard.log 2>&1 &
echo "  pid=$!"

sleep 1

echo "[2/6] correct_pc_transformer (logger 의 lidar_pts 카운트용)"
python3 /home/user/ros2_ws/src/ego-planner-a4vai/correct_pc_transformer.py \
  > ${LOG_DIR}/pc_transformer.log 2>&1 &
echo "  pid=$!"

sleep 2

echo "[3/6] node_att_ctrl"
ros2 run pathfollowing node_att_ctrl > ${LOG_DIR}/att_ctrl.log 2>&1 &
echo "  pid=$!"

echo "[4/6] node_MPPI_output"
ros2 run pathfollowing node_MPPI_output > ${LOG_DIR}/mppi.log 2>&1 &
echo "  pid=$!"

sleep 2

echo "[5/6] path_following_bridge_test (orchestrator + fusion_weight ramp)"
ros2 run algorithm_test path_following_bridge_test \
  > ${LOG_DIR}/orchestrator.log 2>&1 &
echo "  pid=$!"

# orchestrator OFFBOARD started 대기 (takeoff 안전 완료 + fusion ramp 시작)
echo "[ ... waiting for OFFBOARD started ... ]"
until grep -q 'OFFBOARD started' ${LOG_DIR}/orchestrator.log 2>/dev/null; do sleep 1; done

echo ""
echo "[6/6] OFFBOARD 진입 확인 → fusion 0→1.0 자체 ramp (5s) 진행 중"
echo ""
echo "=== pf-only test running. ==="
echo "  Logs: ${LOG_DIR}/"
echo "  CSV:  ${DIAG_CSV}"
echo ""
echo "분석: python3 ${SCRIPT_DIR}/plot_pretty4.py ${DIAG_CSV}"
echo "Stop: pkill -f 'offboard.py|correct_pc_transformer|node_att_ctrl|node_MPPI_output|path_following_bridge_test|flight_logger.py'"

# flight_logger 종료까지 대기 (DURATION 후 자동 종료)
wait
