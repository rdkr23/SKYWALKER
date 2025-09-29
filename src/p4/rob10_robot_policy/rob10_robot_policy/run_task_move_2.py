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
from tf2_ros import TransformListener, Buffer, TransformBroadcaster, StaticTransformBroadcaster

from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState  # <-- Added: actual message on /xarm/joint_states

import tf2_ros
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Int16

from rob10_robot_policy.robots.gen5_move import Gen5ReachPolicy
from p4_robot_controller.transformation_functions import quad_to_tm, tm_to_quad, rpy_to_tm, tm_to_rpy, rpy_to_quat, \
    euler_rpy_to_quat, quat_to_rpy


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

    limits_low = np.array([-6.283185307179586,
                           -2.0589999996956756,
                           -6.283185307179586,
                           -0.19197993453406906,
                           -6.283185307179586,
                           -1.6929698980332752,
                           -6.283185307179586])

    limits_high = np.array([6.283185307179586,
                            2.0943998147821756,
                            6.283185307179586,
                            3.9269998926993517,
                            6.283185307179586,
                            3.141592653589793,
                            6.283185307179586])

    # ROS topics and joint names
    STATE_TOPIC = '/joint_states' #'/xarm/joint_states'
    ACTIVATION_TOPIC = '/rl/activate_base_move'
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
        super().__init__('reach_policy_node_move')

        # Initialize tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot = Gen5ReachPolicy()
        self.target_command = np.zeros(7)
        self.step_size = 1.0 / 100  # 10 ms period = 100 Hz

        self.i = 0
        self.fail_quietly = fail_quietly
        self.verbose = verbose

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.tf_broadcaster = TransformBroadcaster(self)

        # self.pub_freq = 1.0  # Hz
        self.current_pos = None  # Dictionary of current joint positions
        self.target_pos = None  # List of target joint positions

        self.declare_parameter('target_positions', [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.target_positionsss = self.get_parameter('target_positions').value
        self.original_data = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

        self.camera_offset_crd = [0, 0, 0.4, 0.7071068, -0.7071068, 0, 0]

        self.F_world_ee_pos = None
        self.F_world_ee_quat_wxyz = None

        # Subscriber for joint states (sensor_msgs/JointState on /xarm/joint_states)
        self.create_subscription(
            JointState,
            self.STATE_TOPIC,
            self.sub_callback,
            10
        )

        self.timer = None

        self.create_subscription(
            Int16,
            self.ACTIVATION_TOPIC,
            self.activate_model,
            10
        )

        # Publisher for joint trajectory commands
        self.pub = self.create_publisher(JointTrajectory, self.CMD_TOPIC, 100)

        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES  # Must be a list of strings

        point = JointTrajectoryPoint()
        point.positions = [0.01221730, -0.04014257, 0.01221730,
                           0.53929221, 0.01221730, -1.00356287, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)  # OK for ROS 2
        traj.points.append(point)

        self.pub.publish(traj)


        self.min_traj_dur = 1.0  # Minimum trajectory duration in seconds

        self.get_logger().info("ReachPolicy node initialized.")

    def activate_model(self, msg):
        self.target_positionsss = [0.0, -0.1, 0.0, 1.0, 0.0, 0.0, 0.0]

        self.get_logger().info("Activated RL move base")

        transform: TransformStamped = self.tf_buffer.lookup_transform(
            "base_rotated", "eef_rotated", rclpy.time.Time()
        )

        T_BE0_pos = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
        T_BE0_quat_xyzw = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]

        T_WB0_pos = [-0.03445, 0.0047, 0.5019]
        T_WB0_quat_xyzw = [0.0, 0.0, 0.0, 1.0]  # ROS order
        self.F_world_ee_pos, self.F_world_ee_quat_wxyz = self.compute_world_base_pose(T_WB0_pos, T_WB0_quat_xyzw, T_BE0_pos, T_BE0_quat_xyzw)

        self.create_frame()
        self.timer = self.create_timer(self.step_size, self.step_callback)

    def create_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()  # real (sim) time, not 0
        t.header.frame_id = 'eef_rotated'
        t.child_frame_id = 'world_frame_sim'

        t.transform.translation.x = self.F_world_ee_pos[0]
        t.transform.translation.y = self.F_world_ee_pos[1]
        t.transform.translation.z = self.F_world_ee_pos[2]
        t.transform.rotation.w = self.F_world_ee_quat_wxyz[0]
        t.transform.rotation.x = self.F_world_ee_quat_wxyz[1]
        t.transform.rotation.y = self.F_world_ee_quat_wxyz[2]
        t.transform.rotation.z = self.F_world_ee_quat_wxyz[3]

        self.static_broadcaster.sendTransform(t)  # <-- static once

    def compute_world_base_pose(self,
            T_WB0_pos, T_WB0_quat_xyzw,  # base in world at t0 (ROS xyzw)
            T_BE0_pos, T_BE0_quat_xyzw,  # ee in base at t0 (ROS xyzw)
            current_BE_pos=None, current_BE_quat_xyzw=None,  # ee in base at t (ROS xyzw)
            current_EB_pos=None, current_EB_quat_xyzw=None  # base in ee at t (ROS xyzw)
    ):
        # Convert ROS xyzw -> scipy Rotation directly
        R_WB0 = R.from_quat(T_WB0_quat_xyzw)
        R_BE0 = R.from_quat(T_BE0_quat_xyzw)

        # Build transforms
        T_WB0 = np.eye(4)
        T_WB0[:3, :3] = R_WB0.as_matrix()
        T_WB0[:3, 3] = T_WB0_pos

        T_BE0 = np.eye(4)
        T_BE0[:3, :3] = R_BE0.as_matrix()
        T_BE0[:3, 3] = T_BE0_pos

        # Fixed world->ee
        T_EW = np.linalg.inv(T_BE0) @ np.linalg.inv(T_WB0)

        # Output pos + quaternion in wxyz
        pos = T_EW[:3, 3]
        quat_xyzw = R.from_matrix(T_EW[:3, :3]).as_quat()
        quat_wxyz = np.roll(quat_xyzw, 1)  # xyzw -> wxyz
        return pos, quat_wxyz

    def get_current_base_position(self):
        """
        Get the current end-effector position using tf2_ros.

        Returns:
            np.ndarray: The [x, y, z] position of the end-effector.
        """
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    "world_frame_sim", "base_rotated",
                    rclpy.time.Time(),  # Time(0) = latest  # wait up to 200 ms
                )


                translation = transform.transform.translation
                rotation = transform.transform.rotation

                return np.array([
                    translation.x,
                    translation.y,
                    translation.z,  # Adjust z for the end-effector offset
                    rotation.w,
                    rotation.x,
                    rotation.y,
                    rotation.z
                ])
            except Exception as e:
                self.get_logger().error(f"Could not get transform: {e}")

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

    def step_callback(self):
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

        self.target_command = self.target_positionsss
        # np.array([0.4, 0.0, 0.4, 0.7071, 0, 0.7071, 0])  # Example target command
        # Compute the current end-effector position

        self.create_frame()

        current_ee_dict = self.get_current_base_position()


        if current_ee_dict is None:
            self.get_logger().warn("Skipping step due to missing end-effector position.")
            return

        # Pass the current end-effector position to the robot policy
        self.robot.current_base_position = current_ee_dict #np.concatenate((p_sim, q_sim_wxyz))  # [x, y, z, w, x, y, z]
        # Get simulation joint positions from the robot's forward model
        joint_pos = self.robot.forward(self.step_size, self.target_command)

        if joint_pos is not None:
            if len(joint_pos) != 7:
                raise Exception(f"Expected 7 joint positions, got {len(joint_pos)}!")

            joint_pos_clipped = np.clip(joint_pos, self.limits_low, self.limits_high)

            traj = JointTrajectory()
            traj.joint_names = self.JOINT_NAMES

            point = JointTrajectoryPoint()
            point.positions = joint_pos_clipped.tolist()
            point.time_from_start = Duration(sec=1, nanosec=0)  # Temps pour atteindre la position
            traj.points.append(point)

            self.pub.publish(traj)
        else:
            self.get_logger().info(f"bad stuff...")

        self.i += 1



def main(args=None):
    rclpy.init(args=args)
    node = ReachPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
