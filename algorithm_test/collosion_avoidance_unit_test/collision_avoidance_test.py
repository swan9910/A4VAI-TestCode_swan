# Librarys

# Library for common
import numpy as np
import os

# ROS libraries
import rclpy
from rclpy.node import Node
# ----------------------------------------------------------------------------------------#
# PX4 msgs libraries
from px4_msgs.msg import TrajectorySetpoint

from ..lib.common_fuctions import set_initial_variables, state_logger, publish_to_plotter, BodytoNED, set_wp
from ..lib.publisher import PX4Publisher, HeartbeatPublisher, PlotterPublisher, ModulePublisher
from ..lib.publisher import PubFuncHeartbeat, PubFuncPX4, PubFuncModule, PubFuncPlotter
from ..lib.subscriber import PX4Subscriber, CmdSubscriber, HeartbeatSubscriber
from ..lib.timer import HeartbeatTimer, MainTimer, CommandPubTimer
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
# ----------------------------------------------------------------------------------------#
class CollisionAvoidanceTest(Node):
    def __init__(self):
        super().__init__("collision_avoidance_test")
        # ----------------------------------------------------------------------------------------#
        # region INITIALIZE
        dir = os.path.dirname(os.path.abspath(__file__))
        sim_name = "ca_unit_test"
        set_initial_variables(self, dir, sim_name)

        self.offboard_mode.velocity = True

        self.yaw_cmd_rad = 0.0
        self.vel_ned_cmd_normal = np.zeros(3)
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region PUBLISHERS
        self.pub_px4 = PX4Publisher(self)
        self.pub_px4.declareVehicleCommandPublisher()
        self.pub_px4.declareOffboardControlModePublisher()
        self.pub_px4.declareTrajectorySetpointPublisher()

        self.pub_module = ModulePublisher(self)
        self.pub_module.declareLocalWaypointPublisherToPF()

        self.pub_heartbeat = HeartbeatPublisher(self)
        self.pub_heartbeat.declareControllerHeartbeatPublisher()
        self.pub_heartbeat.declarePathFollowingHeartbeatPublisher()
        self.pub_heartbeat.declarePathPlanningHeartbeatPublisher()

        # self.pub_plotter = PlotterPublisher(self)
        # self.pub_plotter.declareGlobalWaypointPublisherToPlotter()
        # self.pub_plotter.declareLocalWaypointPublisherToPlotter()
        # self.pub_plotter.declareHeadingPublisherToPlotter()
        # self.pub_plotter.declareStatePublisherToPlotter()
        # self.pub_plotter.declareMinDistancePublisherToPlotter()
        # end region
        # ----------------------------------------------------------------------------------------#
        # region PUB FUNC
        self.pub_func_heartbeat = PubFuncHeartbeat(self)
        self.pub_func_px4       = PubFuncPX4(self)
        self.pub_func_module    = PubFuncModule(self)
        self.pub_func_plotter   = PubFuncPlotter(self)
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region SUBSCRIBERS
        self.sub_px4 = PX4Subscriber(self)
        self.sub_px4.declareVehicleLocalPositionSubscriber()
        self.sub_px4.declareVehicleAttitudeSubscriber()

        self.sub_cmd = CmdSubscriber(self)
        # self.sub_cmd.declareCAVelocitySetpointSubscriber()
        self.create_subscription(Twist, '/ca_vel_2_control', self.ca_vel_callback, 1)
        self.rand_point_sub = self.create_subscription(
            Bool,
            "/ca_rand_point_flag",
            self.rand_point_callback,
            1  # best-effort sensor QoS
        )

        self.sub_hearbeat = HeartbeatSubscriber(self)
        self.sub_hearbeat.declareCollisionAvoidanceHeartbeatSubscriber()
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region TIMER
        self.timer_offboard_control = MainTimer(self)
        self.timer_offboard_control.declareOffboardControlTimer(self.offboard_control_main)

        self.timer_cmd = CommandPubTimer(self)
        self.timer_cmd.declareOffboardVelocityControlTimer()

        self.timer_heartbeat = HeartbeatTimer(self)
        self.timer_heartbeat.declareControllerHeartbeatTimer()
        self.timer_heartbeat.declarePathPlanningHeartbeatTimer()
        self.timer_heartbeat.declarePathFollowingHeartbeatTimer()
        # endregion
        self.set_forward_cmd()
    # ----------------------------------------------------------------------------------------#
    # region MAIN CODE
    def offboard_control_main(self):
        # if self.offboard_var.ca_heartbeat == True:

            # send offboard mode and arm mode command to px4
            if self.mode_status.DISARM == True:
                self.mode_status.TAKEOFF = True
                self.mode_status.DISARM = False
                self.get_logger().info('Mode Status : TAKEOFF')

            if self.offboard_var.counter == self.offboard_var.flight_start_time and self.mode_status.TAKEOFF == True:
                # arm cmd to px4
                self.pub_func_px4.publish_vehicle_command(self.modes.prm_arm_mode)
                # offboard mode cmd to px4
                self.pub_func_px4.publish_vehicle_command(self.modes.prm_takeoff_mode)

            # takeoff after a certain period of time
            elif self.offboard_var.counter <= self.offboard_var.flight_start_time:
                self.offboard_var.counter += 1

            # check if the vehicle is ready to initial position
            if self.mode_status.TAKEOFF == True and self.state_var.z > self.guid_var.init_pos[2]:
                self.mode_status.TAKEOFF = False
                self.get_logger().info('Vehicle is reached to initial position')
                self.get_logger().info('Mode Status : OFFBOARD/COLLISION_AVOIDANCE')
                self.mode_status.OFFBOARD = True
                self.mode_status.COLLISION_AVOIDANCE = True

            if self.mode_status.OFFBOARD == True and self.mode_status.COLLISION_AVOIDANCE == True:
                self.offboard_mode.attitude = False
                self.offboard_mode.velocity = True

                # body → NED 변환을 매 루프마다 갱신 (DCM이 업데이트된 후)
                self.veh_vel_set.ned_velocity = BodytoNED(self.veh_vel_set.body_velocity, self.state_var.dcm_b2n)
                self.veh_vel_set.ned_velocity[2] = 0.0

                self.pub_func_px4.publish_offboard_control_mode(self.offboard_mode)
                self.pub_func_px4.publish_vehicle_command(self.modes.prm_offboard_mode)

                
        # state_logger(self)
    # endregion
    # ----------------------------------------------------------------------------------------#
    # region CALCULATION FUNC
    def set_forward_cmd(self):
        self.veh_vel_set.body_velocity = np.array([1, 0, 0])
        self.veh_vel_set.ned_velocity = BodytoNED(self.veh_vel_set.body_velocity, self.state_var.dcm_b2n)
        self.veh_vel_set.yaw = self.ca_var.yaw_0
        self.get_logger().info('yaw: ' + str(self.veh_vel_set.yaw))
    def ca_vel_callback(self, msg):
        vx_command = 3.0  # 고정 (신경망 vx가 1m/s로 작아서)
        vy_gain = 1.0     # ONNX 출력 범위 확대했으므로 gain 1
        yaw_gain = 0.5    # yaw 출력 절반으로 감소 (과도한 회전 방지)

        # 로그 파일 초기화 (첫 콜백에서만)
        if not hasattr(self, '_log_file_initialized'):
            self._log_file_initialized = True
            log_dir = "/home/user/workspace/ros2/logs"
            os.makedirs(log_dir, exist_ok=True)
            self._log_path = os.path.join(log_dir, "ca_commands.csv")
            with open(self._log_path, "w") as f:
                f.write("timestamp,vy_raw,yaw_raw,vx_final,vy_final,yaw_final,rand_point\n")
            self.get_logger().info(f"CA command logging to: {self._log_path}")

        if self.flags.rand_point_flag == True:
            # 회피 완료 후 경로 복귀: vx 유지하면서 측면 속도와 yaw는 0
            self.veh_vel_set.body_velocity = np.array([vx_command, 0.0, 0.0])
            self.veh_vel_set.yawspeed = 0.0
            vx_final, vy_final, yaw_final = vx_command, 0.0, 0.0
        else:
            # 충돌회피 진행 중: vy와 yaw 명령 사용
            # direct_infer.py에서 이미 방향 결정됨 - vy와 yaw 모두 그대로 사용
            vy = -msg.linear.y * vy_gain  # 그대로 사용
            yaw = -msg.angular.z * yaw_gain  # 부호 반전 제거 - vy와 같은 방향으로 회전

            # 안전 제한
            vy = np.clip(vy, -6.0, 6.0)
            yaw = np.clip(yaw, -1.5, 1.5)  # yaw_gain=0.5 적용 후 최대값

            self.veh_vel_set.body_velocity = np.array([vx_command, vy, 0.0])
            self.veh_vel_set.yawspeed = yaw
            vx_final, vy_final, yaw_final = vx_command, vy, yaw

        self.veh_vel_set.ned_velocity = BodytoNED(self.veh_vel_set.body_velocity, self.state_var.dcm_b2n)

        # 고도는 유지
        self.veh_vel_set.ned_velocity[2] = 0.0

        # 파일 로깅
        timestamp = self.get_clock().now().nanoseconds / 1e9
        with open(self._log_path, "a") as f:
            f.write(f"{timestamp:.3f},{msg.linear.y:.4f},{msg.angular.z:.4f},"
                   f"{vx_final:.4f},{vy_final:.4f},{yaw_final:.4f},{int(self.flags.rand_point_flag)}\n")
    def rand_point_callback(self, msg: Bool):
        self.flags.rand_point_flag = msg.data
    # endregion
    # ----------------------------------------------------------------------------------------#

def main(args=None):
    rclpy.init(args=args)
    collision_avoidance_test = CollisionAvoidanceTest()
    rclpy.spin(collision_avoidance_test)
    collision_avoidance_test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
