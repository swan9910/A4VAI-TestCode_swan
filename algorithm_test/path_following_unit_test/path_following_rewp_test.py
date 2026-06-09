# Phase 2: 실제 비행 중 reWP 거동 측정 테스트
#
# 기존 path_following_test 흐름:
#   INIT → 이륙 → OFFBOARD/PATH_FOLLOWING → wp.csv 따라 비행 → 착륙
#
# 본 테스트의 추가:
#   1. fusion_weight 1.0 발행 활성화 (SLERP fusion 동작)
#   2. OFFBOARD 진입 후 일정 간격으로 reWP 시나리오 발행
#   3. MPPI 출력, 자세 setpoint, 위치를 CSV로 기록

# Library for common
import csv
import os
import time
import numpy as np

# ROS libraries
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from ..lib.common_fuctions import set_initial_variables, state_logger, publish_to_plotter, set_wp
from ..lib.timer import HeartbeatTimer, MainTimer, CommandPubTimer
from ..lib.subscriber import PX4Subscriber, FlagSubscriber, CmdSubscriber, EtcSubscriber
from ..lib.publisher import PX4Publisher, HeartbeatPublisher, ModulePublisher, PlotterPublisher
from ..lib.publisher import PubFuncHeartbeat, PubFuncPX4, PubFuncModule, PubFuncPlotter

from custom_msgs.msg import LocalWaypointSetpoint
from px4_msgs.msg import VehicleAttitudeSetpoint
from std_msgs.msg import Float64MultiArray


# ── reWP 시나리오 정의 (path_following_test 기준 NED 또는 ENU 일치) ──
# state_var.z 가 ENU "고도(+Z up)"이므로 NED frame에서 takeoff height는 5
# 첫 시나리오는 wp.csv 와 동일한 초기 경로 - 이륙 후 추적 시작점
REWP_SCENARIOS = [
    {"label": "straight",   "x": [0.0,  20.0, 40.0, 60.0],
                            "y": [0.0,   0.0,  0.0,  0.0],
                            "z": [5.0,   5.0,  5.0,  5.0]},
    {"label": "turn_right", "x": [0.0,  10.0, 20.0, 20.0],
                            "y": [0.0,   0.0, 10.0, 20.0],
                            "z": [5.0,   5.0,  5.0,  5.0]},
    {"label": "turn_left",  "x": [0.0,  10.0, 20.0, 20.0],
                            "y": [0.0,   0.0,-10.0,-20.0],
                            "z": [5.0,   5.0,  5.0,  5.0]},
    {"label": "alt_change", "x": [0.0,  20.0, 40.0, 60.0],
                            "y": [0.0,   0.0,  0.0,  0.0],
                            "z": [5.0,   8.0,  8.0,  5.0]},
    {"label": "diagonal",   "x": [0.0,  15.0, 30.0, 45.0],
                            "y": [0.0,  10.0, 20.0, 30.0],
                            "z": [5.0,   5.0,  5.0,  5.0]},
]

REWP_INTERVAL_SEC   = 5.0          # reWP 발행 주기
REWP_START_DELAY    = 5.0          # OFFBOARD 진입 후 첫 reWP까지 대기

# ── Fusion weight ramp-up 설정 (옵션 A + C) ───────────────────
# 0초에서 0 → FUSION_RAMP_TIME 초에 FUSION_TARGET 까지 선형 증가
# 너무 공격적인 path following 명령으로 추락하지 않도록 점진 적용
FUSION_RAMP_TIME    = 5.0          # 0 → target 까지 시간 (초)
FUSION_TARGET       = 0.5          # 최종 fusion_weight (0~1)

CSV_DIR  = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(CSV_DIR, "rewp_flight_log.csv")


class PathFollowingReWPTest(Node):
    def __init__(self):
        super().__init__("path_following_rewp_test")

        # ── 기존 path_following_test 와 동일한 초기화 ──
        dir = os.path.dirname(os.path.abspath(__file__))
        sim_name = "pf_rewp_test"
        set_initial_variables(self, dir, sim_name)

        # ── PUBLISHERS ───────────────────────────────────────────
        self.pub_px4 = PX4Publisher(self)
        self.pub_px4.declareVehicleCommandPublisher()
        self.pub_px4.declareOffboardControlModePublisher()
        self.pub_px4.declareAttitudeCommandPublisher()
        self.pub_px4.declareFusionWeightPublisher()

        self.pub_module = ModulePublisher(self)
        self.pub_module.declareLocalWaypointPublisherToPF()

        self.pub_heartbeat = HeartbeatPublisher(self)
        self.pub_heartbeat.declareControllerHeartbeatPublisher()
        self.pub_heartbeat.declareCollisionAvoidanceHeartbeatPublisher()
        self.pub_heartbeat.declarePathPlanningHeartbeatPublisher()

        self.pub_plotter = PlotterPublisher(self)
        self.pub_plotter.declareGlobalWaypointPublisherToPlotter()
        self.pub_plotter.declareLocalWaypointPublisherToPlotter()
        self.pub_plotter.declareHeadingPublisherToPlotter()
        self.pub_plotter.declareStatePublisherToPlotter()
        self.pub_plotter.declareMinDistancePublisherToPlotter()

        # ── SUBSCRIBERS ─────────────────────────────────────────
        self.sub_px4 = PX4Subscriber(self)
        self.sub_px4.declareVehicleLocalPositionSubscriber()

        self.sub_cmd = CmdSubscriber(self)
        self.sub_cmd.declarePFAttitudeSetpointSubscriber()

        self.sub_flag = FlagSubscriber(self)
        self.sub_flag.declareConveyLocalWaypointCompleteSubscriber()
        self.sub_flag.declarePFCompleteSubscriber()

        self.sub_etc = EtcSubscriber(self)
        self.sub_etc.declareHeadingWPIdxSubscriber()

        # 추가: MPPI 출력 + 자세 setpoint 기록용 직접 구독
        self.create_subscription(Float64MultiArray, 'MPPI/out/dbl_MPPI',
                                 self._cb_mppi, 10)
        self.create_subscription(VehicleAttitudeSetpoint, '/pf_att_2_control',
                                 self._cb_pf_att, 10)

        # ── PUB FUNC ─────────────────────────────────────────────
        self.pub_func_heartbeat = PubFuncHeartbeat(self)
        self.pub_func_px4       = PubFuncPX4(self)
        self.pub_func_module    = PubFuncModule(self)
        self.pub_func_plotter   = PubFuncPlotter(self)

        # ── TIMER ────────────────────────────────────────────────
        self.timer_offboard_control = MainTimer(self)
        self.timer_offboard_control.declareOffboardControlTimer(self.offboard_control_main)

        self.timer_cmd = CommandPubTimer(self)
        self.timer_cmd.declareAttitudeCommandTimer()
        self.timer_cmd.declareFusionWeightTimer()

        self.timer_heartbeat = HeartbeatTimer(self)
        self.timer_heartbeat.declareControllerHeartbeatTimer()
        self.timer_heartbeat.declarePathPlanningHeartbeatTimer()
        self.timer_heartbeat.declareCollisionAvoidanceHeartbeatTimer()

        # ── reWP 상태 ───────────────────────────────────────────
        self.rewp_active     = False
        self.rewp_idx        = 0           # 다음에 발행할 시나리오 index
        self.rewp_last_time  = 0.0
        self.rewp_offboard_t0 = 0.0        # OFFBOARD 진입 시각

        # ── CSV 로그 초기화 ─────────────────────────────────────
        self.mppi_buf = None    # [Ax, eta, calc_ms]
        self.att_buf  = None    # [q0, q1, q2, q3]
        self.log_records  = []
        self.rewp_events  = []   # (t_sec, label)
        self.t_start_wall = time.time()

        with open(CSV_PATH, 'w', newline='') as f:
            csv.writer(f).writerow([
                't_sec', 'mode', 'fusion_w',
                'pos_x_ned', 'pos_y_ned', 'pos_z_ned',
                'mppi_Ax', 'mppi_eta', 'mppi_calc_ms',
                'q0', 'q1', 'q2', 'q3', 'q_norm',
                'wp_event', 'wp_label',
            ])

        # 50 Hz 로깅 타이머
        self.create_timer(0.02, self._log_tick)

        self.get_logger().info(f'CSV log: {CSV_PATH}')
        self.get_logger().info('▶ Phase 2 reWP test ready — waiting for takeoff')

    # ──────────────────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────────────────
    def _cb_mppi(self, msg):
        if len(msg.data) >= 3:
            self.mppi_buf = list(msg.data[:3])

    def _cb_pf_att(self, msg):
        self.att_buf = list(msg.q_d)

    # ──────────────────────────────────────────────────────────
    # 메인 제어 루프 (기존 path_following_test 흐름 + reWP + fusion publish)
    # ──────────────────────────────────────────────────────────
    def offboard_control_main(self):
        # fusion_weight 계산 + 실제 발행 (원본은 주석 처리됨)
        self.weight_callback()
        self.pub_func_px4.publish_fusion_weight(self.weight)

        if self.mode_status.DISARM == True:
            self.mode_status.TAKEOFF = True
            self.mode_status.DISARM  = False

        if self.offboard_var.counter == self.offboard_var.flight_start_time and self.mode_status.TAKEOFF == True:
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_arm_mode)
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_takeoff_mode)

        elif self.offboard_var.counter <= self.offboard_var.flight_start_time:
            self.offboard_var.counter += 1

        # 이륙 완료 확인 → 첫 waypoint 발행 (기존 동작)
        if self.mode_status.TAKEOFF == True and self.state_var.z > self.guid_var.init_pos[2]:
            self.mode_status.TAKEOFF             = False
            self.flags.pf_get_local_waypoint     = True
            set_wp(self)
            self.pub_func_module.local_waypoint_publish(True)
            publish_to_plotter(self)
            self.get_logger().info('Vehicle is reached to initial position')

        if self.flags.pf_get_local_waypoint == True and self.mode_status.OFFBOARD == False:
            self.mode_status.OFFBOARD       = True
            self.mode_status.PATH_FOLLOWING = True
            self.rewp_offboard_t0 = self._now_rel()
            self.get_logger().info(
                f'★ OFFBOARD started @ t={self.rewp_offboard_t0:.1f}s — '
                f'reWP will start in {REWP_START_DELAY}s')

        if self.mode_status.OFFBOARD == True and self.flags.pf_done == False:
            self.pub_func_px4.publish_offboard_control_mode(self.offboard_mode)
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_offboard_mode)

            # reWP 트리거
            self._maybe_send_rewp()

        if self.flags.pf_done == True and self.mode_status.LANDING == False:
            self.mode_status.OFFBOARD       = False
            self.mode_status.PATH_FOLLOWING = False
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_land_mode)

            if np.abs(self.state_var.vz_n) < 0.05 and np.abs(self.state_var.z < 0.05):
                self.mode_status.LANDING = True
                self.get_logger().info('Vehicle is landed')

        if self.mode_status.LANDING == True and self.mode_status.is_disarmed == False:
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_disarm_mode)
            self.mode_status.is_disarmed = True
            self.get_logger().info('Vehicle is disarmed')
            self._flush_csv()

        state_logger(self)

    # ──────────────────────────────────────────────────────────
    # reWP 발행 로직
    # ──────────────────────────────────────────────────────────
    def _maybe_send_rewp(self):
        t = self._now_rel()
        elapsed_in_offboard = t - self.rewp_offboard_t0

        if elapsed_in_offboard < REWP_START_DELAY:
            return

        if (t - self.rewp_last_time) < REWP_INTERVAL_SEC and self.rewp_last_time > 0:
            return

        if self.rewp_idx >= len(REWP_SCENARIOS):
            return  # 모든 시나리오 끝남

        s = REWP_SCENARIOS[self.rewp_idx]
        self.guid_var.waypoint_x = list(s["x"])
        self.guid_var.waypoint_y = list(s["y"])
        self.guid_var.waypoint_z = list(s["z"])

        # path_planning_complete = False ⇒ reWP 트리거
        # (단 첫 발행은 True 로 하여 path following을 초기화)
        is_first = (self.rewp_idx == 0)
        self.pub_func_module.local_waypoint_publish(is_first)

        self.rewp_events.append((t, s["label"]))
        self.rewp_last_time = t
        flag = 'INIT' if is_first else 'reWP'
        self.get_logger().info(f'[{t:.1f}s] WP → {s["label"]} ({flag})')
        self.rewp_idx += 1

    def weight_callback(self):
        self.weight.timestamp = int(Clock().now().nanoseconds / 1000)
        if self.mode_status.OFFBOARD == False:
            self.weight.fusion_weight = 0.0
        else:
            # OFFBOARD 진입 후 경과 시간 기준 선형 램프업
            t_offboard = self._now_rel() - self.rewp_offboard_t0
            ramp = max(0.0, min(1.0, t_offboard / FUSION_RAMP_TIME))
            self.weight.fusion_weight = float(ramp * FUSION_TARGET)

    # ──────────────────────────────────────────────────────────
    # CSV 로깅
    # ──────────────────────────────────────────────────────────
    def _now_rel(self):
        return time.time() - self.t_start_wall

    def _log_tick(self):
        if self.mppi_buf is None or self.att_buf is None:
            return
        t  = self._now_rel()
        q  = self.att_buf
        qn = float(np.sqrt(sum(v*v for v in q)))

        wp_event, wp_label = 0, ''
        for ev_t, ev_label in self.rewp_events:
            if abs(t - ev_t) < 0.025:
                wp_event, wp_label = 1, ev_label

        mode = ('LAND'   if self.mode_status.LANDING else
                'OFFBOARD' if self.mode_status.OFFBOARD else
                'TAKEOFF'  if self.mode_status.TAKEOFF else
                'INIT')

        self.log_records.append([
            round(t, 4), mode,
            round(float(self.weight.fusion_weight), 4),
            round(self.state_var.x, 3) if hasattr(self.state_var, 'x') else 0.0,
            round(self.state_var.y, 3) if hasattr(self.state_var, 'y') else 0.0,
            round(self.state_var.z, 3) if hasattr(self.state_var, 'z') else 0.0,
            round(self.mppi_buf[0], 6),
            round(self.mppi_buf[1], 6),
            round(self.mppi_buf[2], 4),
            round(q[0], 6), round(q[1], 6), round(q[2], 6), round(q[3], 6),
            round(qn, 6),
            wp_event, wp_label,
        ])
        if len(self.log_records) >= 100:
            self._flush_csv()

    def _flush_csv(self):
        if not self.log_records:
            return
        with open(CSV_PATH, 'a', newline='') as f:
            csv.writer(f).writerows(self.log_records)
        self.log_records.clear()


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingReWPTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._flush_csv()
        node.get_logger().info(f'CSV saved: {CSV_PATH}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
