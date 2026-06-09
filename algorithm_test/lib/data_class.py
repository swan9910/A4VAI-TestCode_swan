import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.clock import Clock
class StateVariable:
    def __init__(self):
        # ned position [m]
        self.x = 0.
        self.y = 0.
        self.z = 0.
        # ned velocity [m/s]
        self.vx_n = 0.
        self.vy_n = 0.
        self.vz_n = 0.
        # body velocity [m/s]
        self.vx_b = 0.
        self.vy_b = 0.
        self.vz_b = 0.
        # attitude euler angle [rad]
        self.roll = 0.
        self.pitch = 0.
        self.yaw = 0.
        # attitude quaternion
        self.q = [0., 0., 0., 0.]

        # DCM matrix
        self.dcm_b2n = np.zeros((3, 3))
        self.dcm_n2b = np.zeros((3, 3))
        
class CollisionAvoidanceVariable:
    def __init__(self):
        self.clock = Clock()
        self.bridge = CvBridge()
        self.depth_min_distance = 0.
        self.lidar_min_distance = 0.
        self.lidar_counter = 0
        self.sign = 0.
        self.time = self.clock.now()
        self.yaw_0 = 0*np.pi/180.

        # Collision Avoidance Velocity Command Offset
        self.vx_offset = -1.130069137
        self.vy_offset = -0.08785345405340195
        self.vz_offset = 0.39943796396255493
        self.yawrate_offset = 0.13739442825317383

class OffboardVariable:
    def __init__(self):
        self.counter = 0
        self.flight_start_time = 10
        self.period_heartbeat = 1
        self.period_offboard_control = 0.2     # required about 5Hz for attitude control (proof that the external controller is healthy
        self.period_offboard_att_ctrl = 0.004  # required 250Hz at least for attitude control
        self.period_offboard_vel_ctrl = 0.02
        self.period_fusion_weight = 0.03
        self.pf_heartbeat = False
        self.pp_heartbeat = False
        self.ca_heartbeat = False
        self.ct_heartbeat = False

class GuidVariable:
    def __init__(self):
        self.init_pos = np.array([0., 0., 4.0])
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_z = []
        self.cur_wp = 0
        self.wp_distance = 0.5
        self.real_wp_x = []
        self.real_wp_y = []
        self.real_wp_z = []
        

class ModeStatus:
    def __init__(self):
        self.DISARM                 = True 
        self.TAKEOFF                = False
        self.OFFBOARD               = False
        self.COLLISION_AVOIDANCE    = False
        self.PATH_FOLLOWING         = False
        self.LANDING                = False

class Flags:
    def __init__(self):
        self.path_planning             = False
        self.pf_get_local_waypoint  = False
        self.pf_done                = False
        self.obstacle_flag          = False
        self.rand_point_flag        = False

class SimulationVariable:
    def __init__(self, sim_name, dir):
        self.sim_name = sim_name
        self.dir = dir
        self.flight_log = None

# offboard control mode
class OffboardControlModeState:
    def __init__(self):
        self.position = False
        self.velocity = True
        self.acceleration = False
        self.attitude = False
        self.body_rate = False

class VehicleCommandState:
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_DO_SET_MODE = 176

    def __init__(self):
        self.CMD_mode = np.nan
        self.params = np.nan * np.ones(7)

class CommandFactory:
    @staticmethod
    def create_arm_command(arm: bool) -> VehicleCommandState:
        """Create an arm or disarm command."""
        command = VehicleCommandState()
        command.CMD_mode = VehicleCommandState.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command.params[0] = 1 if arm else 0
        return command

    @staticmethod
    def create_mode_command(base_mode: int, custom_mode: int, sub_mode: int = np.nan) -> VehicleCommandState:
        """Create a mode command."""
        command = VehicleCommandState()
        command.CMD_mode = VehicleCommandState.VEHICLE_CMD_DO_SET_MODE
        command.params[0] = 1
        command.params[1] = base_mode
        command.params[2] = custom_mode
        if not np.isnan(sub_mode):
            command.params[3] = sub_mode
        return command

class VehicleModes:
    def __init__(self):
        self.prm_arm_mode = CommandFactory.create_arm_command(arm=True)
        self.prm_disarm_mode = CommandFactory.create_arm_command(arm=False)

        self.prm_offboard_mode = CommandFactory.create_mode_command(base_mode=6, custom_mode=0)
        self.prm_takeoff_mode = CommandFactory.create_mode_command(base_mode=4, custom_mode=2)
        self.prm_land_mode = CommandFactory.create_mode_command(base_mode=4, custom_mode=6)
        self.prm_position_mode = CommandFactory.create_mode_command(base_mode=3, custom_mode=0)

# # .. variable - vehicle attitude setpoint
class VehicleAttitudeSetpointState:
    def __init__(self):
        self.roll_body  = np.nan  # body angle in NED frame (can be NaN for FW)
        self.pitch_body = np.nan  # body angle in NED frame (can be NaN for FW)
        self.yaw_body   = np.nan  # body angle in NED frame (can be NaN for FW)
        self.q_d = [np.nan, np.nan, np.nan, np.nan]
        self.yaw_sp_move_rate = np.nan  # rad/s (commanded by user)
        # For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
        # For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
        self.thrust_body = np.nan * np.ones(3)  # Normalized thrust command in body NED frame [-1,1]

class VehicleVelocitySetpointState:
    def __init__(self):
        self.position = [np.NaN, np.NaN, np.NaN]
        self.ned_velocity = [0.0, 0.0, 0.0]
        self.acceleration = [np.NaN, np.NaN, np.NaN]
        self.jerk = [np.NaN, np.NaN, np.NaN]

        self.yaw = np.nan
        self.yawspeed = np.nan

        self.body_velocity = np.nan * np.ones(3)

        # Collision avoidance initial velocity (captured when CA starts)
        self.ca_initial_vx = 0.0
        self.ca_start_time = None  # CA 진입 시간 (ramping 용)
        self.ca_ramp_duration = 1.5  # seconds (vx ramping 시간 - fusion weight와 동기화)
        self.ca_ramp_delay = 0.0  # seconds (즉시 회피 - 중앙 장애물 대응)