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

# Goal = wp1+wp2 chained 마지막 wp 의 ENU 변환: PF NED (x=north -128.05, y=east -97.52, alt 18.50)
# → ENU (east=-97.52, north=-128.05, up=18.50)  [S128m, W98m, alt+18.5m]
GOAL_X=-97.52
GOAL_Y=-128.05
GOAL_Z=18.50
TAKEOFF_ALT=8.0
DURATION=500

# pf wp.csv: 새 경로 (10 wps, PF NED 좌표, 북서로 flip + alt +8.3044, 첫·마지막 wp 제거)
WP_CSV=/home/user/a4vai_ws/install/algorithm_test/lib/python3.12/site-packages/algorithm_test/path_following_unit_test/wp.csv
cat > ${WP_CSV} <<EOF
x,y,z
0.0000, 0.0000, 3.0000
0.0000, 0.0000, 4.3221
-24.8836, -11.9611, 3.0000
-50.8152, -12.0277, 1.5310
-76.1901, -18.4120, -0.6725
-102.1570, -18.0781, -3.6105
-127.8025, -21.3919, -5.5202
-154.5996, -11.6341, -8.7520
-180.4691, -12.4052, -10.2210
-205.7733, -19.5926, -10.6617
-231.1812, -25.6032, -11.6900
-231.1812, -25.6032, -11.8369
-219.4956, -33.3961, -11.8369
-207.7645, -41.1490, -11.8369
-195.9407, -48.8204, -11.5175
-182.8210, -55.3538, -11.1981
-169.0258, -61.2942, -10.2399
-156.2773, -68.1536, 0.9391
-149.4763, -80.2359, 16.5897
-135.5684, -86.0773, 18.1867
-128.0468, -97.5268, 18.5061
EOF
echo "wp.csv set: wp1+wp2 chained (21 wps, wp1 +3 alt offset, takeoff_alt=8), end NED(-128.05, -97.53, alt +18.51) = ENU(-97.53, -128.05, alt +18.51)"

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

echo "[1/3] offboard.py + correct_pc + ego_planner + mode_switcher + pf 노드 + wp_publisher (전부 parallel)"
nohup python3 /home/user/ros2_ws/src/ego-planner-a4vai/offboard_integration.py \
  --ros-args -p takeoff_alt:=${TAKEOFF_ALT} -p yaw_offset_deg:=0.0 \
  -p goal_x:=${GOAL_X} -p goal_y:=${GOAL_Y} -p goal_z:=${GOAL_Z} \
  > ${LOG_DIR}/offboard.log 2>&1 &
nohup python3 /home/user/ros2_ws/src/ego-planner-a4vai/correct_pc_transformer.py \
  > ${LOG_DIR}/pc_transformer.log 2>&1 &
nohup ros2 launch ego_planner airsim_px4.launch.py \
  map_size_x:=600.0 map_size_y:=600.0 map_size_z:=50.0 \
  max_vel:=3.0 \
  > ${LOG_DIR}/ego_planner.log 2>&1 &
nohup python3 ${SCRIPT_DIR}/mode_switcher.py \
  --dist-ca-enter 6.0 --dist-pf-enter 10.0 \
  --rate 20 --pf-cap 1.0 --ramp-time 0.0 \
  > ${LOG_DIR}/mode_switcher.log 2>&1 &
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

echo "[2/3] goal — service call (retry until success)"
GOAL_SENT=0
for ATTEMPT in 1 2 3 4 5 6 7 8 9 10; do
  if timeout 5 ros2 service call /ego_planner/set_goal traj_utils/srv/SetGoal \
       "{goal: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}}" \
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
