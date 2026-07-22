#!/bin/bash
# C++ ego_planner + pf integration with mode-switching
# 검증된 C++ ego_planner 사용 + 우리 mode_switcher (lidar 기반) + pf 통합
#
# 흐름:
#   offboard.py → drone OFFBOARD + /planning/pos_cmd 따라가기 (CA mode 시)
#   C++ ego_planner_node + traj_server → /planning/pos_cmd 발행
#   pf 노드들 (att_ctrl + MPPI) → /pf_att_2_control 발행
#   mode_switcher.py → lidar 거리 보고 fusion_weight 결정 (장애물 가까우면 ego, 멀면 pf)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# bind-mount 안에 로그 두면 host 에서 실시간 보임 (/home/ercuam/A4VAI-Algorithms-ROS2/logs/integration_cpp/)
LOG_DIR=/home/user/a4vai_ws/logs/integration_cpp
FLIGHT_LOG_DIR=/home/user/a4vai_ws/logs/flight_csv
mkdir -p ${LOG_DIR} ${FLIGHT_LOG_DIR}
rm -f ${LOG_DIR}/*.log

source /opt/ros/jazzy/setup.bash
source /home/user/realgazebo/RealGazebo-ROS2/install/setup.bash 2>/dev/null || true
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true
source /home/user/a4vai_ws/install/setup.bash 2>/dev/null || true

export PATH=/usr/local/cuda-12.8/bin:${PATH}
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH}

# Goal / TAKEOFF / DURATION — env 로 override 가능 (PP 통합 시 외부에서 export)
GOAL_X=${GOAL_X:--112.79}
GOAL_Y=${GOAL_Y:--117.85}
GOAL_Z=${GOAL_Z:--4.84}
TAKEOFF_ALT=${TAKEOFF_ALT:-5.0}
DURATION=${DURATION:-500}

# wp.csv: SKIP_WP_GEN=1 이면 skip (외부에서 미리 작성 가정)
WP_CSV=/home/user/a4vai_ws/install/algorithm_test/lib/python3.12/site-packages/algorithm_test/path_following_unit_test/wp.csv
if [ "${SKIP_WP_GEN:-0}" != "1" ]; then
cat > ${WP_CSV} <<EOF
x,y,z
0.0000, 0.0000, 5.0000
0.0000, 0.0000, 5.0000
-17.3697, -10.2920, 4.7062
-35.0698, -19.2401, 3.0903
-54.1561, -22.5470, 1.1806
-73.9839, -22.8366, -1.4636
-93.0444, -26.2484, -3.0795
-112.6889, -27.2837, -5.2830
-134.7347, -18.5480, -7.6334
-153.1217, -24.7004, -8.8086
-170.4380, -35.2100, -9.5431
-170.4380, -35.2100, -9.5431
-170.4380, -35.2100, -9.5431
-153.5227, -38.7280, -9.6900
-144.3345, -45.8070, -9.6900
-132.3471, -51.5960, -9.3962
-130.8883, -62.2368, -9.2493
-128.8653, -72.6178, -9.1024
-126.6444, -82.9074, -8.8086
-124.5776, -93.2681, -8.6617
-119.5584, -102.2683, -8.2210
-117.8508, -112.7945, -4.6954
-117.8508, -112.7945, -4.8423
EOF
echo "wp.csv set: default PSO result (23 wps)"
else
echo "wp.csv: skip generation (external write), using existing ${WP_CSV}"
fi

DIAG_CSV=${FLIGHT_LOG_DIR}/$(date +%Y%m%d_%H%M%S)_integration_cpp.csv

echo "[0/3] flight_logger + keepalive + auto-arm (parallel)"
nohup python3 ${SCRIPT_DIR}/../lib/flight_logger.py \
  --duration ${DURATION} --csv ${DIAG_CSV} \
  > ${LOG_DIR}/flight_logger.log 2>&1 &
nohup bash -c 'while true; do
  ros2 topic pub --once /vehicle1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
    "{command: 176, param1: 1.0, param2: 6.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}" > /dev/null 2>&1
  sleep 1
done' > ${LOG_DIR}/offboard_keepalive.log 2>&1 &

# auto-arm: offboard mode + arming_state 가 2 (armed) 될 때까지 매 2s 마다 둘 다 명령. 90s 후 give-up.
nohup bash -c '
  for i in $(seq 1 45); do
    sleep 2
    state=$(timeout 4 ros2 topic echo --once --qos-reliability best_effort \
      /vehicle1/fmu/out/vehicle_status_v1 2>/dev/null | grep "^arming_state" | head -1 | awk "{print \$2}")
    if [ "$state" = "2" ]; then
      echo "[auto-arm] armed at attempt $i (state=$state)"
      exit 0
    fi
    echo "[auto-arm] attempt $i: state=$state, retry offboard+arm..."
    ros2 topic pub --once /vehicle1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
      "{command: 176, param1: 1.0, param2: 6.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}" > /dev/null 2>&1
    sleep 0.3
    ros2 topic pub --once /vehicle1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
      "{command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 255, source_component: 0, from_external: true}" > /dev/null 2>&1
  done
  echo "[auto-arm] FAILED after 45 attempts"
' > ${LOG_DIR}/auto_arm.log 2>&1 &

echo "[1a/3] offboard.py + correct_pc (이륙만 먼저)"
nohup python3 /home/user/ros2_ws/src/ego-planner-a4vai/offboard_integration.py \
  --ros-args -p takeoff_alt:=${TAKEOFF_ALT} -p yaw_offset_deg:=0.0 \
  -p goal_x:=${GOAL_X} -p goal_y:=${GOAL_Y} -p goal_z:=${GOAL_Z} \
  > ${LOG_DIR}/offboard.log 2>&1 &
nohup python3 /home/user/ros2_ws/src/ego-planner-a4vai/correct_pc_transformer.py \
  > ${LOG_DIR}/pc_transformer.log 2>&1 &

# 이륙 완료 대기 — offboard.py 가 STATE_PLANNER_READY 진입 시 "Ready! Pos(ENU)" 로그
echo "  이륙 대기..."
TAKEOFF_TIMEOUT=120
TAKEOFF_DONE=0
for i in $(seq 1 ${TAKEOFF_TIMEOUT}); do
  sleep 1
  if grep -q "Ready! Pos(ENU)" ${LOG_DIR}/offboard.log 2>/dev/null; then
    echo "  ★ 이륙 완료 (${i}s)"
    TAKEOFF_DONE=1; break
  fi
done
if [ ${TAKEOFF_DONE} -eq 0 ]; then
  echo "  ✗ 이륙 timeout (${TAKEOFF_TIMEOUT}s) — PF/CA 그대로 진행"
fi
sleep 2   # PLANNER_READY 후 hover 안정화

echo "[1b/3] ego_planner + mode_switcher + pf 노드 + wp_publisher (PF/CA 활성화)"
nohup ros2 launch ego_planner airsim_px4.launch.py \
  map_size_x:=600.0 map_size_y:=600.0 map_size_z:=50.0 \
  max_vel:=3.0 \
  > ${LOG_DIR}/ego_planner.log 2>&1 &
nohup python3 ${SCRIPT_DIR}/mode_switcher.py \
  --dist-ca-enter 8.0 --dist-pf-enter 10.0 \
  --rate 20 --pf-cap 1.0 --ramp-time 0.5 \
  --takeoff-alt ${TAKEOFF_ALT} \
  > ${LOG_DIR}/mode_switcher.log 2>&1 &
nohup python3 ${SCRIPT_DIR}/../lib/flight_streamer.py \
  --ip 100.68.0.70 --port 45680 --rate 10.0 \
  > ${LOG_DIR}/flight_streamer.log 2>&1 &

nohup python3 ${SCRIPT_DIR}/mode_broadcaster.py \
  --ip 100.68.0.70 --port 45678 --rate 1.0 \
  > ${LOG_DIR}/mode_broadcaster.log 2>&1 &
nohup ros2 run pathfollowing node_att_ctrl > ${LOG_DIR}/att_ctrl.log 2>&1 &
nohup ros2 run pathfollowing node_MPPI_output > ${LOG_DIR}/mppi.log 2>&1 &
nohup python3 ${SCRIPT_DIR}/pf_attitude_smoother.py \
  --tau-att 0.3 --tau-thrust 0.2 \
  > ${LOG_DIR}/smoother.log 2>&1 &
nohup python3 ${SCRIPT_DIR}/wp_publisher.py \
  --wp-csv ${WP_CSV} --repeat 5 --period 1.0 \
  > ${LOG_DIR}/wp_publisher.log 2>&1 &
nohup python3 ${SCRIPT_DIR}/pc_accumulator.py \
  > ${LOG_DIR}/pc_accumulator.log 2>&1 &

sleep 5   # ego_planner_node bring-up 대기

# ego_planner 초기 goal = wp[2] (PSO 첫 진짜 wp). final goal 향한 직진 회피.
# ca_advance_wp 가 heading_wp_idx 따라 자동으로 다음 wp 로 update.
INIT_LINE=$(sed -n "4p" ${WP_CSV} | tr -d " ")
INIT_GX=$(echo $INIT_LINE | cut -d, -f1)
INIT_GY=$(echo $INIT_LINE | cut -d, -f2)
INIT_GZ=$(echo $INIT_LINE | cut -d, -f3)
echo "  초기 ego goal = wp[2] = ($INIT_GX, $INIT_GY, $INIT_GZ)  (final goal: $GOAL_X,$GOAL_Y,$GOAL_Z 는 ca_advance_wp 가 update)"

echo "[2/3] goal — service call (retry until success)"
GOAL_SENT=0
for ATTEMPT in 1 2 3 4 5 6 7 8 9 10; do
  if timeout 5 ros2 service call /ego_planner/set_goal traj_utils/srv/SetGoal \
       "{goal: {x: ${INIT_GX}, y: ${INIT_GY}, z: ${INIT_GZ}}}" \
       >> ${LOG_DIR}/goal_pub.log 2>&1; then
    if grep -q "goal accepted" ${LOG_DIR}/goal_pub.log; then
      echo "  ★ goal accepted (attempt ${ATTEMPT})"
      GOAL_SENT=1; break
    fi
  fi
  echo "  attempt ${ATTEMPT} failed, retry..." >> ${LOG_DIR}/goal_pub.log
  sleep 1
done
[ ${GOAL_SENT} -eq 0 ] && echo "  ✗ goal send failed after 10 attempts"
echo "  pid=$!"

echo ""
echo "=== integration-cpp running ==="
echo "  Logs: ${LOG_DIR}/"
echo "  CSV:  ${DIAG_CSV}"
echo ""
echo "Stop: pkill -f 'offboard.py|correct_pc|airsim_px4|ego_planner_node|traj_server|node_att_ctrl|node_MPPI_output|smoother|mode_switcher|flight_logger|while true|ros2 topic pub'"

wait
