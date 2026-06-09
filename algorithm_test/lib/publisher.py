# common libraries
import numpy as np
import time
# ROS2 libraries
from rclpy.clock import Clock


# PX4 message libraries
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import PathFollowingAttCmd
from px4_msgs.msg import FusionWeight
# Custom message libraries
from custom_msgs.msg import LocalWaypointSetpoint
from custom_msgs.msg import GlobalWaypointSetpoint
from custom_msgs.msg import StateFlag
# ROS2 libraries
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

#-------------------------------------------------------------------------------------------#
# region: PUBLISHER CLASSES
# px4 publisher
class PX4Publisher:
    def __init__(self, node):
        self.node = node

    # declare vehicle command publisher
    def declareVehicleCommandPublisher(self):
        self.node.vehicle_command_publisher = self.node.create_publisher(
            VehicleCommand, 
            "/vehicle1/fmu/in/vehicle_command",
            self.node.qos_profile_px4
        )

    # declare offboard control mode publisher
    def declareOffboardControlModePublisher(self):
        self.node.offboard_control_mode_publisher = self.node.create_publisher(
            OffboardControlMode,
            "/vehicle1/fmu/in/offboard_control_mode",
            self.node.qos_profile_px4
        )
    
    # publisher for vehicle velocity setpoint
    def declareTrajectorySetpointPublisher(self):
        self.node.trajectory_setpoint_publisher = self.node.create_publisher(
            TrajectorySetpoint,
            "/vehicle1/fmu/in/trajectory_setpoint",
            self.node.qos_profile_px4
        )
    def declareAttitudeCommandPublisher(self):
        self.node.attitude_command_publisher = self.node.create_publisher(
            PathFollowingAttCmd,
            "/vehicle1/fmu/in/path_following_att_cmd",
            self.node.qos_profile_px4
        )
    def declareFusionWeightPublisher(self):
        self.node.fusion_weight_publisher = self.node.create_publisher(
            FusionWeight,
            '/vehicle1/fmu/in/fusion_weight',
            self.node.qos_profile_px4
        )

# Moduile Data publisher
class ModulePublisher:
    def __init__(self, node):
        self.node = node

    # declare local waypoint publisher to path following
    def declareLocalWaypointPublisherToPF(self):
        self.node.local_waypoint_publisher_to_pf = self.node.create_publisher(
            LocalWaypointSetpoint,
            "/local_waypoint_setpoint_to_PF",
            1
        )
    
    # declare mode flag publisher to collision checker
    def declareModeFlagPublisherToCC(self):
        self.node.mode_status_publisher_to_cc = self.node.create_publisher(
            StateFlag,
            '/mode_flag_to_CC',
            1
        )
    
    # declare vehicle mode publisher
    def declareVehicleModePublisher(self):
        self.node.vehicle_mode_publisher = self.node.create_publisher(
            Bool,
            "/vehicle_mode",
            1
        )

# heartbeat publisher
class HeartbeatPublisher:
    def __init__(self, node):
        self.node = node

    # declare controller heartbeat publisher
    def declareControllerHeartbeatPublisher(self):
        self.node.controller_heartbeat_publisher = self.node.create_publisher(
            Bool,
            "/controller_heartbeat",
            1
        )

    # declare path planning heartbeat publisher
    def declarePathPlanningHeartbeatPublisher(self):
        self.node.path_planning_heartbeat_publisher = self.node.create_publisher(
            Bool,
            "/path_planning_heartbeat",
            1
        )

    # declare collision avoidance heartbeat publisher
    def declareCollisionAvoidanceHeartbeatPublisher(self):
        self.node.collision_avoidance_heartbeat_publisher = self.node.create_publisher(
            Bool,
            "/collision_avoidance_heartbeat",
            1
        )

    # declare path following heartbeat publisher
    def declarePathFollowingHeartbeatPublisher(self):
        self.node.path_following_heartbeat_publisher = self.node.create_publisher(
            Bool,
            "/path_following_heartbeat",
            1
        )

# plotter publisher
class PlotterPublisher:
    def __init__(self, node):
        self.node = node

    # declare global waypoint publisher to plotter
    def declareGlobalWaypointPublisherToPlotter(self):
        self.node.global_waypoint_publisher_to_plotter = self.node.create_publisher(
            GlobalWaypointSetpoint,
            "/global_waypoint_setpoint_to_plotter",
            1
        )
    
    # declare local waypoint publisher to plotter
    def declareLocalWaypointPublisherToPlotter(self):
        self.node.local_waypoint_publisher_to_plotter = self.node.create_publisher(
            LocalWaypointSetpoint,
            "/local_waypoint_setpoint_to_plotter",
            1
        )

    # declare heading publisher to plotter
    def declareHeadingPublisherToPlotter(self):
        self.node.heading_publisher_to_plotter = self.node.create_publisher(
            Float32,
            "/heading",
            1
        )

    # declare control mode publisher to plotter
    def declareStatePublisherToPlotter(self):
        self.node.control_mode_publisher_to_plotter = self.node.create_publisher(
            Bool,
            "/controller_state",
            1
        )

    # declare min distance publisher to plotter
    def declareMinDistancePublisherToPlotter(self):
        self.node.min_distance_publisher_to_plotter = self.node.create_publisher(
            Float64MultiArray,
            "/min_distance",
            1
        )
# endregion
#-------------------------------------------------------------------------------------------#
# region: PUBLISH FUNCTIONS
# px4 publish functions
class PubFuncPX4:
    def __init__(self, node):
        self.node = node

    # publish offboard control mode
    def publish_offboard_control_mode(self, offboard_mode):
        msg = OffboardControlMode()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        msg.position        = offboard_mode.position
        msg.velocity        = offboard_mode.velocity
        msg.acceleration    = offboard_mode.acceleration
        msg.attitude        = offboard_mode.attitude
        msg.body_rate       = offboard_mode.body_rate
        self.node.offboard_control_mode_publisher.publish(msg)

    # publish_vehicle_command
    def publish_vehicle_command(self, modes):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        msg.param1  = modes.params[0]
        msg.param2  = modes.params[1]
        msg.param3  = modes.params[2]
        msg.command = modes.CMD_mode
        # values below are in [3]
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.node.vehicle_command_publisher.publish(msg)

    # publish attitude offboard command
    def publish_vehicle_attitude_setpoint(self, mode_flag, veh_att_set):
        if mode_flag.PATH_FOLLOWING:
            msg = VehicleAttitudeSetpoint()
            msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
            msg.roll_body           = veh_att_set.roll_body
            msg.pitch_body          = veh_att_set.pitch_body
            msg.yaw_body            = veh_att_set.yaw_body
            msg.yaw_sp_move_rate    = veh_att_set.yaw_sp_move_rate
            msg.q_d[0]              = veh_att_set.q_d[0]
            msg.q_d[1]              = veh_att_set.q_d[1]
            msg.q_d[2]              = veh_att_set.q_d[2]
            msg.q_d[3]              = veh_att_set.q_d[3]
            msg.thrust_body[0]      = 0.0
            msg.thrust_body[1]      = 0.0
            msg.thrust_body[2]      = veh_att_set.thrust_body[2]
            self.node.vehicle_attitude_setpoint_publisher.publish(msg)

    # publish vehicle velocity setpoint
    def publish_vehicle_velocity_setpoint(self, mode_flag, veh_vel_set):
        if mode_flag.COLLISION_AVOIDANCE == True:
            msg = TrajectorySetpoint()
            msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
            msg.position        = veh_vel_set.position
            msg.acceleration    = veh_vel_set.acceleration
            msg.jerk            = veh_vel_set.jerk
            msg.velocity        = np.float32([veh_vel_set.ned_velocity[0], veh_vel_set.ned_velocity[1], veh_vel_set.ned_velocity[2]])
            msg.yaw             = veh_vel_set.yaw
            msg.yawspeed        = veh_vel_set.yawspeed

            self.node.trajectory_setpoint_publisher.publish(msg)

    # publish attitude command
    def publish_att_command(self, veh_att_set):
        msg = PathFollowingAttCmd()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds

        # check data is nan
        if np.isnan(veh_att_set.q_d[0]):
            msg.pf_q_cmd[0]              = 0.0
        else:
            msg.pf_q_cmd[0]              = veh_att_set.q_d[0]
        if np.isnan(veh_att_set.q_d[1]):
            msg.pf_q_cmd[1]              = 0.0
        else:
            msg.pf_q_cmd[1]              = veh_att_set.q_d[1]
        if np.isnan(veh_att_set.q_d[2]):
            msg.pf_q_cmd[2]              = 0.0
        else:
            msg.pf_q_cmd[2]              = veh_att_set.q_d[2]
        if np.isnan(veh_att_set.q_d[3]):
            msg.pf_q_cmd[3]              = 0.0
        else:
            msg.pf_q_cmd[3]              = veh_att_set.q_d[3]

        if np.isnan(veh_att_set.thrust_body[0]):
            msg.pf_thrust_cmd[0]      = 0.0
        else:
            msg.pf_thrust_cmd[0]      = veh_att_set.thrust_body[0]
        if np.isnan(veh_att_set.thrust_body[1]):
            msg.pf_thrust_cmd[1]      = 0.0
        else:
            msg.pf_thrust_cmd[1]      = veh_att_set.thrust_body[1]
        if np.isnan(veh_att_set.thrust_body[2]):
            msg.pf_thrust_cmd[2]      = 0.0
        else:
            msg.pf_thrust_cmd[2]      = veh_att_set.thrust_body[2]


        self.node.attitude_command_publisher.publish(msg)
    
    def publish_fusion_weight(self, weight):
        msg = FusionWeight()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        msg.fusion_weight = float(weight.fusion_weight)
        self.node.fusion_weight_publisher.publish(msg)



# module data publish functions
class PubFuncModule:
    def __init__(self, node):
        self.node = node
        self.guid_var = node.guid_var
        self.mode_status = node.mode_status
    # publish local waypoint
    def local_waypoint_publish(self, flag):
        msg = LocalWaypointSetpoint()
        msg.path_planning_complete = flag
        msg.waypoint_x = self.guid_var.waypoint_x
        msg.waypoint_y = self.guid_var.waypoint_y
        msg.waypoint_z = self.guid_var.waypoint_z

        self.node.local_waypoint_publisher_to_pf.publish(msg)

    # publish global waypoint
    def global_waypoint_publish(self, publisher):
        msg = GlobalWaypointSetpoint()
        msg.start_point = [self.guid_var.waypoint_x[0], self.guid_var.waypoint_z[0], self.guid_var.waypoint_y[0]]
        msg.goal_point = [self.guid_var.waypoint_x[-1], self.guid_var.waypoint_z[-1], self.guid_var.waypoint_y[-1]]
        publisher.publish(msg)

    def publish_flags(self):
        msg = StateFlag()
        msg.disarm = self.mode_status.DISARM
        msg.takeoff = self.mode_status.TAKEOFF
        msg.path_following = self.mode_status.PATH_FOLLOWING
        msg.collision_avoidance = self.mode_status.COLLISION_AVOIDANCE
        msg.offboard = self.mode_status.OFFBOARD
        msg.landing = self.mode_status.LANDING
        
        # for debugging
        # self.node.get_logger().info(f"publish_flags: {msg.path_following}, {msg.collision_avoidance}, {msg.offboard}, {msg.landing}")
        
        self.node.mode_status_publisher_to_cc.publish(msg)

    # publish vehicle mode
    def publish_vehicle_mode(self):
        msg = Bool()
        if self.node.mode_status.COLLISION_AVOIDANCE == True:
            msg.data = True
        if self.node.mode_status.PATH_FOLLOWING == True:
            msg.data = False
        self.node.vehicle_mode_publisher.publish(msg)

# heartbeat publish functions
class PubFuncHeartbeat:
    def __init__(self, node):
        self.node = node

    # publish collision avoidance heartbeat
    def publish_collision_avoidance_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.node.collision_avoidance_heartbeat_publisher.publish(msg)

    # publish path planning heartbeat
    def publish_path_planning_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.node.path_planning_heartbeat_publisher.publish(msg)

    # publish controller heartbeat
    def publish_controller_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.node.controller_heartbeat_publisher.publish(msg)

    # publish path following heartbeat
    def publish_path_following_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.node.path_following_heartbeat_publisher.publish(msg)

# plotter publish functions
class PubFuncPlotter:
    def __init__(self, node):
        self.node = node

    # publish heading
    def publish_heading(self, state_var):
        msg = Float32()
        msg.data = float(state_var.yaw)
        self.node.heading_publisher_to_plotter.publish(msg)

    # publish control mode
    def publish_control_mode(self, mode_flag):
        msg = Bool()
        if mode_flag.COLLISION_AVOIDANCE == True:
            msg.data = True
        else:
            msg.data = False
        self.node.control_mode_publisher_to_plotter.publish(msg)

    # publish obstacle min distance
    def publish_obstacle_min_distance(self, ca_var):
        msg = Float64MultiArray()
        msg.data = [float(ca_var.depth_min_distance), float(ca_var.lidar_min_distance)]
        self.node.min_distance_publisher_to_plotter.publish(msg)

    # publish global waypoint to plotter
    def publish_global_waypoint_to_plotter(self, guid_var):
        msg = GlobalWaypointSetpoint()
        msg.start_point = [0., guid_var.init_pos[2], 0.]
        msg.goal_point = [guid_var.waypoint_x[-1], guid_var.waypoint_z[-1], guid_var.waypoint_y[-1]]
        self.node.global_waypoint_publisher_to_plotter.publish(msg)

    # publish local waypoint to plotter
    def publish_local_waypoint_to_plotter(self, guid_var):
        msg = LocalWaypointSetpoint()
        msg.path_planning_complete = True
        # Publish only remaining waypoints from cur_wp onwards
        if guid_var.cur_wp < len(guid_var.waypoint_x):
            msg.waypoint_x = guid_var.waypoint_x[guid_var.cur_wp:]
            msg.waypoint_y = guid_var.waypoint_y[guid_var.cur_wp:]
            msg.waypoint_z = guid_var.waypoint_z[guid_var.cur_wp:]
        else:
            msg.waypoint_x = []
            msg.waypoint_y = []
            msg.waypoint_z = []
        self.node.local_waypoint_publisher_to_plotter.publish(msg)
# endregion
#-------------------------------------------------------------------------------------------#