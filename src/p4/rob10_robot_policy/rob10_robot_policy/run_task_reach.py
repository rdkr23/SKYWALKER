#!/usr/bin/env python3
"""
run_task_reach.py
--------------------

ROS 2 node that drives a Kinova Gen3 arm with a simple
open-loop “reach” policy.

* Subscribes to: /joint_trajectory_controller/state
* Publishes to : /joint_trajectory_controller/joint_trajectory
* Runs at      : 100 Hz

Author: Louis Le Lay
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node


from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformListener, Buffer, TransformBroadcaster

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState  # <-- Added: actual message on /xarm/joint_states

import tf2_ros
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Int16, Bool

from rob10_robot_policy.robots.gen4_ptp import Gen4ReachPolicy
from p4_robot_controller.transformation_functions import quad_to_tm, tm_to_quad, rpy_to_tm, tm_to_rpy, rpy_to_quat, euler_rpy_to_quat, quat_to_rpy

def quat_mul(a,b):
    aw,ax,ay,az = a
    bw,bx,by,bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw
    ])


class ReachPolicy(Node):
    """ROS2 node for controlling a Gen3 robot's reach policy."""
    
    # Define simulation degree-of-freedom angle limits: (Lower limit, Upper limit, Inversed flag)
    SIM_DOF_ANGLE_LIMITS = [
        (math.degrees(-6.283185307179586), math.degrees(6.283185307179586), False),
        (math.degrees(-2.0589999996956756), math.degrees(2.0943998147821756), False),
        (math.degrees(-6.283185307179586), math.degrees(6.283185307179586), False),
        (math.degrees(-0.19197993453406906), math.degrees(3.9269998926993517), False),
        (math.degrees(-6.283185307179586), math.degrees(6.283185307179586), False),
        (math.degrees(-1.6929698980332752), math.degrees(3.141592653589793), False),
        (math.degrees(-6.283185307179586), math.degrees(6.283185307179586), False),
    ]



    limits_low  = np.array([-6.283185307179586,
         -2.0589999996956756,
         -6.283185307179586,
         -0.19197993453406906,
         -6.283185307179586,
         -1.6929698980332752,
         -6.283185307179586])
     
    limits_high = np.array([ 6.283185307179586,
          2.0943998147821756,
          6.283185307179586,
          3.9269998926993517,
          6.283185307179586,
          3.141592653589793,
          6.283185307179586])

    # ROS topics and joint names
    STATE_TOPIC = '/xarm/joint_states'
    RESPONSE_TOPIC = '/rl/response_p2p'
    # CMD_TOPIC = '/joint_trajectory_controller/joint_trajectory'
    CMD_TOPIC = '/xarm7_traj_controller/joint_trajectory'

    # ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    #     joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    #     points: [
    #         { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
    #     ]
    # }" -1

    JOINT_NAMES = [
        'joint1',
        'joint2',
        'joint3',
        'joint4',
        'joint5',
        'joint6',
        'joint7',
    ]
    
    
    # Mapping from joint name to simulation action index
    JOINT_NAME_TO_IDX = {
        'joint1': 0,
        'joint2': 1,
        'joint3': 2,
        'joint4': 3,
        'joint5': 4,
        'joint6': 5,
        'joint7': 6
    }

    def __init__(self, fail_quietly: bool = False, verbose: bool = False):
        """Initialize the ReachPolicy node."""
        super().__init__('reach_policy_node')

        
         # Initialize tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        
        self.robot = Gen4ReachPolicy()
        self.target_command = np.zeros(7)
        self.step_size = 1.0 / 100  # 10 ms period = 100 Hz

        self.i = 0
        self.fail_quietly = fail_quietly
        self.verbose = verbose

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

        # self.pub_freq = 1.0  # Hz
        self.current_pos = None  # Dictionary of current joint positions
        self.target_pos = None   # List of target joint positions

        self.declare_parameter('target_positions',[0.2, 0.0, 0.5, 0.0 , 0.7071, 0.0, 0.7071])
        self.target_positionsss = self.get_parameter('target_positions').value

        self.camera_offset_crd = [0, 0, 0.25, 1, 0, 0, 0] #0.7071068, -0.7071068, 0, 0

        # Subscriber for joint states (sensor_msgs/JointState on /xarm/joint_states)
        self.create_subscription(
            JointState,
            self.STATE_TOPIC,
            self.sub_callback,
            10
        )

        self.timer = None
        self.pose_reached = True

        self.goal_reached_resp = self.create_publisher(
            Bool,
            self.RESPONSE_TOPIC,
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=100  # Used as a fallback for RMWs that require depth
        )

        self.create_subscription(
            Int16,
            '/rl/activate_p2p',
            self.activate_model,
            qos_profile
        )
        
        # Publisher for joint trajectory commands
        self.pub = self.create_publisher(JointTrajectory, self.CMD_TOPIC, 1)
        self.min_traj_dur = 1.0  # Minimum trajectory duration in seconds
        
        self.get_logger().info("ReachPolicy node initialized.")


    def create_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_base'
        t.child_frame_id = 'tag_pose_frame'

        t.transform.translation.x = self.target_positionsss[0]
        t.transform.translation.y = self.target_positionsss[1]
        t.transform.translation.z = self.target_positionsss[2]
        t.transform.rotation.x = self.target_positionsss[4]
        t.transform.rotation.y = self.target_positionsss[5]
        t.transform.rotation.z = self.target_positionsss[6]
        t.transform.rotation.w = self.target_positionsss[3]

        self.tf_broadcaster.sendTransform(t)

    def get_target_location(self, tag_id):
        while 1:
            try:
                if tag_id is None:
                    return
                transform = self.tf_buffer.lookup_transform('link_base', f'base-tag36h11:{tag_id}',
                                                            rclpy.time.Time())
                self.get_logger().info("Transform reached.")
                self.pose_reached = False

                at_translation = transform.transform.translation
                at_rotation = transform.transform.rotation

                ap = [at_translation.x, at_translation.y, at_translation.z, at_rotation.x, at_rotation.y, at_rotation.z,
                      at_rotation.w]

                tm_base_tag = quad_to_tm(ap)

                tm_tag_offset = quad_to_tm(self.camera_offset_crd)

                tm_base_tag = np.dot(tm_base_tag, tm_tag_offset)

                self.target_positionsss = tm_to_quad(tm_base_tag)
                pos_copy = self.target_positionsss.copy()
                # self.target_positionsss[2] = self.target_positionsss[2] - 0.1
                self.target_positionsss[3] = pos_copy[6]
                self.target_positionsss[4] = pos_copy[3]
                self.target_positionsss[5] = pos_copy[4]
                self.target_positionsss[6] = pos_copy[5]

                break

            except Exception as e:
                self.pose_reached = True
                self.get_logger().info("Target position is zero, not activating.")
                self.get_logger().error(f'Could not get transforms: {e}')

    def activate_model(self, msg):
        self.pose_reached = False
        self.get_logger().info("Getting target pose.")
        self.get_target_location(msg.data)

        self.get_logger().info("Starting...")
        if (self.target_positionsss[0]**2 + self.target_positionsss[1]**2 + self.target_positionsss[2]**2)**0.5 < 0.1:
            self.pose_reached = True
            self.get_logger().info("Target position is zero, not activating.")

        self.timer = self.create_timer(self.step_size, self.step_callback)
        self.timer_controller = self.create_timer(0.1, self.position_reach_determiner)
        self.get_logger().info(f"Timers created. {self.pose_reached}")

    def get_current_ee_position(self):
        """
        Get the current end-effector position using tf2_ros.

        Returns:
            np.ndarray: The [x, y, z] position of the end-effector.
        """
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "link_base", "link_eef", rclpy.time.Time()
            )

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            # q_s = np.array([rotation.w, rotation.x, rotation.y, rotation.z])
            # q_delta = np.array([0.0, 1.0, 0.0, 0.0])  # 180° about x
            #
            # q_t = quat_mul(q_delta, q_s)


            return np.array([
                translation.x,
                translation.y,
                translation.z,   # Adjust z for the end-effector offset
                rotation.w,
                rotation.x,
                rotation.y,
                rotation.z
            ])
        except Exception as e:
            self.get_logger().error(f"Could not get transform: {e}")
        return None

    def sub_callback(self, msg: JointState):
        """
        Callback for receiving joint states and updating current positions.
        Incoming topic /xarm/joint_states uses names like 'joint1'..'joint7'.
        We map those into this node's JOINT_NAMES order 'joint_1'..'joint_7'
        without changing the public names/constants used elsewhere.
        """
        in_names = list(msg.name) if msg.name else []
        in_pos = list(msg.position) if msg.position else []
        in_vel = list(msg.velocity) if msg.velocity else []
        # Pad velocities if missing/short
        if len(in_vel) < len(in_pos):
            in_vel += [0.0] * (len(in_pos) - len(in_vel))

        # Build mapping from incoming names to their indices
        name_to_idx = {n: i for i, n in enumerate(in_names)}

        # Helper to convert 'joint_1' -> 'joint1' (the incoming style)
        def out_to_in(n_out: str) -> str:
            # Only replace the first underscore that separates base name and index
            # 'joint_1' -> 'joint1'
            if n_out.startswith('joint_'):
                return 'joint' + n_out[len('joint_'):]
            return n_out

        # Create ordered outputs matching JOINT_NAMES
        ordered_pos = []
        ordered_vel = []
        actual_pos_dict = {}

        for n_out in self.JOINT_NAMES:
            n_in = out_to_in(n_out)
            if n_in in name_to_idx:
                k = name_to_idx[n_in]
                p = in_pos[k] if k < len(in_pos) else float('nan')
                v = in_vel[k] if k < len(in_vel) else 0.0
            else:
                p = float('nan')  # indicate missing joint
                v = 0.0
            ordered_pos.append(p)
            ordered_vel.append(v)
            # Keep your external keys as 'joint_1'.. for compatibility
            actual_pos_dict[n_out] = p

        # Store dictionary of current joint positions (keys = your JOINT_NAMES)
        self.current_pos = actual_pos_dict

        # Update the robot's state with arrays ordered as JOINT_NAMES
        self.robot.update_joint_state(ordered_pos, ordered_vel)

    def map_joint_angle(self, pos: float, index: int) -> float:
        """
        Map a simulation joint angle (in radians) to the real-world servo angle (in radians).
        """
        L, U, inversed = self.SIM_DOF_ANGLE_LIMITS[index]
        A, B = self.SERVO_ANGLE_LIMITS[index]
        angle_deg = np.rad2deg(float(pos))
        if not L <= angle_deg <= U:
            self.get_logger().warn(
                f"Simulation joint {index} angle ({angle_deg}) out of range [{L}, {U}]. Clipping."
            )
            angle_deg = np.clip(angle_deg, L, U)
        mapped = (angle_deg - L) * ((B - A) / (U - L)) + A
        if inversed:
            mapped = (B - A) - (mapped - A) + A
        if not A <= mapped <= B:
            raise Exception(
                f"Mapped joint {index} angle ({mapped}) out of servo range [{A}, {B}]."
            )
        return mapped

    def position_reach_determiner(self):
        transform: TransformStamped = self.tf_buffer.lookup_transform(
            "link_eef", "tag_pose_frame", rclpy.time.Time()
        )

        translation = transform.transform.translation
        orientation = transform.transform.rotation

        translation_length = math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)
        orientation_magnitude = math.sqrt(orientation.x**2 + orientation.y**2 + orientation.z**2)

        if translation_length < 0.05 and abs(orientation_magnitude) < 0.1:
            self.pose_reached = True

            resp = Bool()
            resp.data = True
            self.get_logger().info("Goal reached, stopping RL p2p.")
            self.goal_reached_resp.publish(resp)

            if self.timer_controller is not None:
                self.timer_controller.cancel()
                self.timer_controller = None

            if self.timer is not None:
                self.timer.cancel()
                self.timer = None



    def step_callback(self):
        if self.pose_reached:
            return

        """
        Timer callback to compute and publish the next joint trajectory command.
        """
        # Example 3D target positions for the XARM7
        # if self.i % 3000 < 1000:
        #     self.target_command = np.array([0.5, 0.2, 0.3])  # Target position in 3D space
        # elif self.i % 3000 < 2000:
        #     self.target_command = np.array([0.4, -0.15, 0.5])
        # else:
        #     self.target_command = np.array([0.6, 0.1, 0.4])


        # SHOULD be of the [x , y ,z w, x, y, z] W before the orinetation 
        self.target_command = self.target_positionsss #[0.3, 0.2, 0.4, 0.0, 0.707, 0.0, 0.707]
        #np.array([0.4, 0.0, 0.4, 0.7071, 0, 0.7071, 0])  # Example target command
        self.create_frame()
        # Compute the current end-effector position
        current_ee_dict = self.get_current_ee_position()
        if current_ee_dict is None:
            self.get_logger().warn("Skipping step due to missing end-effector position.")
            return

        current_ee_pose = current_ee_dict[0:3]
        current_ee_quat = current_ee_dict[3:7]
        #
        #
        # # --- Fixed ROS->Sim transform (world) and constant tool rotation (all given in wxyz) ---
        R_fix_wxyz  = np.array([0.9974949, 0.0007000, 0.0682997, -0.0183999], dtype=float)
        # t_ros2sim   = np.array([-0.034, 0.004, 0.359], dtype=float)
        Q_tool_wxyz = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        #
        #
        # # SciPy uses xyzw; convert once
        #
        R_fix  = R.from_quat(np.r_[R_fix_wxyz[1:],  R_fix_wxyz[0]])
        Q_tool = R.from_quat(np.r_[Q_tool_wxyz[1:], Q_tool_wxyz[0]])
        #
        # # --- Map ROS EE pose -> Sim EE pose ---
        #
        # # position: rotate then translate
        # p_sim = R_fix.apply(current_ee_pose) + t_ros2sim
        #
        # # orientation: world-frame rotation first, then constant tool rotation
        #
        q_ros_xyzw = np.r_[current_ee_quat[1:], current_ee_quat[0]]
        q_sim_xyzw = (R_fix * R.from_quat(q_ros_xyzw) * Q_tool).as_quat()
        q_sim_wxyz = np.r_[q_sim_xyzw[3], q_sim_xyzw[:3]]  # back to wxyz


        # Pass the current end-effector position to the robot policy
        self.robot.current_ee_position = np.concatenate((current_ee_pose, q_sim_wxyz))#self.get_current_ee_position() #np.concatenate((p_sim, q_sim_wxyz))  # [x, y, z, w, x, y, z]
        # Get simulation joint positions from the robot's forward model
        joint_pos = self.robot.forward(self.step_size, self.target_command)

        if joint_pos is not None:
            if len(joint_pos) != 7:
                raise Exception(f"Expected 7 joint positions, got {len(joint_pos)}!")
            
            joint_pos_clipped=np.clip(joint_pos, self.limits_low, self.limits_high)
            
            traj = JointTrajectory()
            traj.joint_names = self.JOINT_NAMES

            point = JointTrajectoryPoint()
            point.positions = joint_pos_clipped.tolist()
            point.time_from_start = Duration(sec=1, nanosec=0)  # Temps pour atteindre la position

            traj.points.append(point)

            if not self.pose_reached:
                self.pub.publish(traj)
            
        self.i += 1



def main(args=None):
    rclpy.init(args=args)
    node = ReachPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
