# Phase 4 통합 테스트:
#   ego-planner → bridge → path following → SLERP fusion → PX4
#
# 기존 path_following_rewp_test 와 다른 점:
#   - reWP 시나리오 자체 발행 안 함 (bridge 가 담당)
#   - wp.csv 의 초기 waypoint 만 publish (path following 기동용)
#   - fusion_weight 0 → 0.5 선형 램프 (이전 검증된 안전값)
#   - 로깅은 동일 (CSV)
#
# 외부에서 함께 띄울 것:
#   1. offboard.py            (이륙 + ego_odom)
#   2. pointcloud_transformer (lidar → world)
#   3. ego_planner stack       (planning)
#   4. ego_to_pf_bridge.py     (waypoint 변환)
#   5. node_att_ctrl           (path following 자세 제어)
#   6. node_MPPI_output        (MPPI 가이던스)
#   7. 본 노드                (이 파일)

import csv
import os
import time
import numpy as np

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


# ── Fusion weight ramp ──────────────────────────────────────────
FUSION_RAMP_TIME    = 5.0
FUSION_TARGET       = 1.0     # path following 100% (이전 0.5는 영향력 부족)

CSV_DIR  = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(CSV_DIR, "bridge_flight_log.csv")


class PathFollowingBridgeTest(Node):
    def __init__(self):
        super().__init__("path_following_bridge_test")

        dir = os.path.dirname(os.path.abspath(__file__))
        sim_name = "pf_bridge_test"
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

        # 로깅용 직접 구독
        self.create_subscription(Float64MultiArray, 'MPPI/out/dbl_MPPI', self._cb_mppi, 10)
        self.create_subscription(VehicleAttitudeSetpoint, '/pf_att_2_control', self._cb_pf_att, 10)
        # bridge 가 발행한 waypoint 수신 카운트
        self.create_subscription(LocalWaypointSetpoint, '/local_waypoint_setpoint_to_PF',
                                 self._cb_bridge_wp, 10)

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

        # ── 상태 ─────────────────────────────────────────────────
        self.t_start_wall    = time.time()
        self.offboard_t0     = 0.0
        self.bridge_wp_count = 0

        # 로깅
        self.mppi_buf = None
        self.att_buf  = None
        self.records  = []
        with open(CSV_PATH, 'w', newline='') as f:
            csv.writer(f).writerow([
                't_sec', 'mode', 'fusion_w',
                'pos_x_ned', 'pos_y_ned', 'pos_z_ned',
                'mppi_Ax', 'mppi_eta', 'mppi_calc_ms',
                'q0', 'q1', 'q2', 'q3', 'q_norm',
                'bridge_wp_total',
            ])
        self.create_timer(0.02, self._log_tick)

        self.get_logger().info(f'CSV: {CSV_PATH}')
        self.get_logger().info('▶ Bridge integration test ready — will use ego-planner waypoints via bridge')

    # ──────────────────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────────────────
    def _cb_mppi(self, msg):
        if len(msg.data) >= 3:
            self.mppi_buf = list(msg.data[:3])

    def _cb_pf_att(self, msg):
        self.att_buf = list(msg.q_d)

    def _cb_bridge_wp(self, msg):
        self.bridge_wp_count += 1

    # ──────────────────────────────────────────────────────────
    # offboard 메인 (path_following_rewp_test 의 takeoff+landing 흐름)
    # ──────────────────────────────────────────────────────────
    def offboard_control_main(self):
        self.weight_callback()
        # fusion_weight 발행은 mode_switcher 가 담당 (옵션 A 아키텍처)
        # self.pub_func_px4.publish_fusion_weight(self.weight)

        if self.mode_status.DISARM == True:
            self.mode_status.TAKEOFF = True
            self.mode_status.DISARM  = False

        if self.offboard_var.counter == self.offboard_var.flight_start_time and self.mode_status.TAKEOFF == True:
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_arm_mode)
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_takeoff_mode)
        elif self.offboard_var.counter <= self.offboard_var.flight_start_time:
            self.offboard_var.counter += 1

        # 이륙 완료 → 초기 waypoint 발행 (wp.csv 의 단순 직선)
        # 이후 bridge 가 ego-planner 출력으로 덮어씀
        if self.mode_status.TAKEOFF == True and self.state_var.z > self.guid_var.init_pos[2]:
            self.mode_status.TAKEOFF             = False
            self.flags.pf_get_local_waypoint     = True
            set_wp(self)
            self.pub_func_module.local_waypoint_publish(True)   # path following 기동
            publish_to_plotter(self)
            self.get_logger().info('Takeoff complete — bridge will override waypoints')

        if self.flags.pf_get_local_waypoint == True and self.mode_status.OFFBOARD == False:
            self.mode_status.OFFBOARD       = True
            self.mode_status.PATH_FOLLOWING = True
            self.offboard_t0                = self._now_rel()
            self.get_logger().info(
                f'★ OFFBOARD started @ t={self.offboard_t0:.1f}s — fusion_weight ramp 시작')

        if self.mode_status.OFFBOARD == True and self.flags.pf_done == False:
            self.pub_func_px4.publish_offboard_control_mode(self.offboard_mode)
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_offboard_mode)

        if self.flags.pf_done == True and self.mode_status.LANDING == False:
            self.mode_status.OFFBOARD       = False
            self.mode_status.PATH_FOLLOWING = False
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_land_mode)
            if np.abs(self.state_var.vz_n) < 0.05 and np.abs(self.state_var.z < 0.05):
                self.mode_status.LANDING = True
                self.get_logger().info('Landed')

        if self.mode_status.LANDING == True and self.mode_status.is_disarmed == False:
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_disarm_mode)
            self.mode_status.is_disarmed = True
            self.get_logger().info('Disarmed')
            self._flush_csv()

        state_logger(self)

    # ──────────────────────────────────────────────────────────
    # fusion weight ramp
    # ──────────────────────────────────────────────────────────
    def weight_callback(self):
        self.weight.timestamp = int(Clock().now().nanoseconds / 1000)
        if self.mode_status.OFFBOARD == False:
            self.weight.fusion_weight = 0.0
        else:
            t_off = self._now_rel() - self.offboard_t0
            ramp  = max(0.0, min(1.0, t_off / FUSION_RAMP_TIME))
            self.weight.fusion_weight = float(ramp * FUSION_TARGET)

    # ──────────────────────────────────────────────────────────
    # 로깅
    # ──────────────────────────────────────────────────────────
    def _now_rel(self):
        return time.time() - self.t_start_wall

    def _log_tick(self):
        if self.mppi_buf is None or self.att_buf is None:
            return
        t = self._now_rel()
        q = self.att_buf
        qn = float(np.sqrt(sum(v*v for v in q)))

        mode = ('LAND'    if self.mode_status.LANDING else
                'OFFBOARD' if self.mode_status.OFFBOARD else
                'TAKEOFF'  if self.mode_status.TAKEOFF else
                'INIT')

        self.records.append([
            round(t, 4), mode,
            round(float(self.weight.fusion_weight), 4),
            round(self.state_var.x, 3),
            round(self.state_var.y, 3),
            round(self.state_var.z, 3),
            round(self.mppi_buf[0], 6),
            round(self.mppi_buf[1], 6),
            round(self.mppi_buf[2], 4),
            round(q[0], 6), round(q[1], 6), round(q[2], 6), round(q[3], 6),
            round(qn, 6),
            self.bridge_wp_count,
        ])
        if len(self.records) >= 100:
            self._flush_csv()

    def _flush_csv(self):
        if not self.records:
            return
        with open(CSV_PATH, 'a', newline='') as f:
            csv.writer(f).writerows(self.records)
        self.records.clear()


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingBridgeTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._flush_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
