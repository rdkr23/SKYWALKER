#!/usr/bin/env python3
"""
Impedance‑planner node for xArm – cleaned up & made ROS 2‑friendly.
The main fixes compared with the previous snippet are:
  • pass exactly three Euler angles to `rpy_to_quat()`
  • initialise every state variable before first use
  • remove typos (`curreent_*` → `cart_vel_*`)
  • add default force/torque masks
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped, PoseStamped
from xarm_msgs.msg import RobotMsg

from p4_robot_controller.transformation_functions import (
    quad_to_tm, rpy_to_tm, rpy_to_quat, tm_to_rpy,
)

# -----------------------------------------------------------------------------
#  Default topic / service names – override with ROS 2 parameters if you like
# -----------------------------------------------------------------------------

DEFAULT_TOPIC_FORCE        = "/xarm/uf_ftsensor_ext_states"
DEFAULT_TOPIC_POSE         = "/xarm/robot_states"
DEFAULT_TOPIC_CART_VEL     = "/xarm/cartesian_velocity"
DEFAULT_TOPIC_POSE_OFFSET  = "/xarm/impedance_offset"
DEFAULT_TOPIC_WRENCH_FILT  = "/xarm/filtered_force"
DEFAULT_SERVICE_SET_STATE  = "/xarm/set_state"

# -----------------------------------------------------------------------------
#  Helper
# -----------------------------------------------------------------------------

def stamp_to_sec(st):
    """ROS 2 builtin_interfaces/Time → seconds as float."""
    return st.sec + st.nanosec * 1e-9


# -----------------------------------------------------------------------------
#  Main node
# -----------------------------------------------------------------------------

class ImpedancePlanner(Node):
    def __init__(self):
        super().__init__("impedance_planner")

        # ─────────────────── parameters ───────────────────
        self.declare_parameters(
            "",
            [
                ("mass",      [2.0, 2.0, 2.0, 0.05, 0.05, 0.05]),
                ("damping",   [50.0, 50.0, 50.0, 2.0, 2.0, 2.0]),
                ("stiffness", [500.0, 500.0, 500.0, 8.0, 8.0, 8.0]),
                ("topic_force",       DEFAULT_TOPIC_FORCE),
                ("topic_robot_pose",  DEFAULT_TOPIC_POSE),
                ("topic_cart_vel",    DEFAULT_TOPIC_CART_VEL),
                ("topic_pose_offset", DEFAULT_TOPIC_POSE_OFFSET),
                ("topic_wrench_filt", DEFAULT_TOPIC_WRENCH_FILT),
                ("service_set_state", DEFAULT_SERVICE_SET_STATE),
            ],
        )

        self.M = np.asarray(self.get_parameter("mass").value, dtype=float)
        self.B = np.asarray(self.get_parameter("damping").value, dtype=float)
        self.K = np.asarray(self.get_parameter("stiffness").value, dtype=float)

        # ─────────────────── internal state ───────────────────
        self.goal_pose      = None  # [x y z R P Y]
        self.last_pose      = None
        self.last_force     = np.zeros(3)
        self.last_torque    = np.zeros(3)

        self.force_filtered  = np.zeros(3)
        self.torque_filtered = np.zeros(3)

        self.force_mask   = [1, 1, 1]  # enable/disable axes if needed
        self.torque_mask  = [0, 0, 0]

        self.current_crd        = np.zeros(3)
        self.current_crd_dot    = np.zeros(3)
        self.current_angle      = np.zeros(3)
        self.current_angle_dot  = np.zeros(3)
        self.current_angle_delta= np.zeros(3)

        self.cart_vel_lin = np.zeros(3)
        self.cart_vel_ang = np.zeros(3)

        self.prev_time = None
        self.activated = False


        # ─────────────────── pubs / subs ───────────────────
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sub_force = self.create_subscription(
            WrenchStamped,
            self.get_parameter("topic_force").value,
            self.cb_force,
            qos_sensor,
        )

        self.sub_pose = self.create_subscription(
            RobotMsg,
            self.get_parameter("topic_robot_pose").value,
            self.cb_robot_pose,
            10,
        )



        self.pub_pose_offset = self.create_publisher(
            PoseStamped,
            self.get_parameter("topic_pose_offset").value,
            10,
        )
        self.pub_wrench_filt = self.create_publisher(
            WrenchStamped,
            self.get_parameter("topic_wrench_filt").value,
            10,
        )
        self.pub_debug_array = self.create_publisher(Float64MultiArray, '/xarm/debug_array',qos_sensor)

        # high‑rate timer (1 kHz)
        self.timer = self.create_timer(0.001, self.update)
        

        self.get_logger().info(
            "ImpedancePlanner ready – waiting for first /robot_states to activate",
        )

    # ───────────────────────────────── callbacks ──────────────────────────────

    def cb_robot_pose(self, msg: RobotMsg):
        # msg.pose is Float32[6]: [x, y, z, R, P, Y]
        
        self.last_pose = np.asarray(msg.pose, dtype=float)
        self.pose_m = self.last_pose[0:3]/1000
        self.rpy = self.last_pose[3:6]
        self.last_pose = np.concatenate((self.pose_m, self.rpy ))
        if not self.activated:
            self.activate_once()

    def cb_force(self, msg: WrenchStamped):
        self.time = stamp_to_sec(msg.header.stamp)
        self.last_force  = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ])
        self.last_torque = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ])
        # simple dead‑zone



    # ───────────────────────────────── helpers ────────────────────────────────

    def activate_once(self):
        """Grab first pose and initialise state trajectories."""
        if self.last_pose is None:
            return

        self.goal_pose = self.last_pose.copy()

        self.current_crd         = np.zeros(3)
        self.current_crd_dot     = np.zeros(3)
        self.current_angle       = self.goal_pose[3:6].copy()
        self.current_angle_dot   = np.zeros(3)
        self.current_angle_delta = np.zeros(3)
        self.prev_time = None
        self.time = None

        self.activated = True
        self.get_logger().info(f"Activated at pose {self.goal_pose}")

    # ───────────────────────────────── control loop ───────────────────────────

    def update(self):
        if not self.activated or self.last_pose is None:
            return  # nothing to do yet

        
        current_time = self.time

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        if dt <= 0.0 or dt > 0.05:        # >5 cycles is a glitch at 100 Hz
            dt = 0.01    
        self.prev_time = current_time

        current_pose = self.last_pose  # [x y z R P Y]

        # ── rotation / transform helpers ────────────────────────────────────
        quat = rpy_to_quat(current_pose[3:6])                   # length‑4
        tm   = np.asarray(quad_to_tm([0, 0, 0, *quat]))        # 4×4 – zero translation
        R    = tm[:3, :3]                                      # extract rotation only

        # ── raw wrench → base frame & filtering ────────────────────────────
        force  = R @ self.last_force
        torque =  self.last_torque

        force  = force  * self.force_mask
        torque = torque * self.torque_mask

        alpha = 0.3  # 1st‑order IIR low‑pass
        self.force_filtered  = alpha * force  + (1 - alpha) * self.force_filtered
        self.torque_filtered = alpha * torque + (1 - alpha) * self.torque_filtered

        # ── translational impedance ─────────────────────────────────────────
        
        
        offset_linear      = self.current_crd - self.goal_pose[0:3]
        current_crd_ddot   = (
            self.force_filtered - self.B[0:3] * self.current_crd_dot - self.K[0:3] * offset_linear
        ) / self.M[0:3]

        self.current_crd_dot += current_crd_ddot * dt
        self.current_crd     += self.current_crd_dot * dt

        self.coordinat= self.current_crd[0:3]*1000

        # ── rotational impedance (Euler‑XYZ) ────────────────────────────────
        offset_angular      = self.current_angle_delta
        current_angle_ddot  = (
            self.torque_filtered - self.B[3:6] * self.current_angle_dot - self.K[3:6] * offset_angular
        ) / self.M[3:6]

        self.current_angle_dot   += current_angle_ddot * dt
        self.current_angle_delta += self.current_angle_dot * dt

        tm_goal_R = rpy_to_tm([0, 0, 0, *self.goal_pose[3:6]])
        tm_total  = tm_goal_R @ rpy_to_tm([0, 0, 0, *self.current_angle_delta])
        self.current_angle = np.asarray(tm_to_rpy(tm_total))[3:6]

        # ── compose output pose ─────────────────────────────────────────────
        pose = np.concatenate((self.coordinat, self.current_angle))



        
        msg = Float64MultiArray()
        msg.data = [pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]]
        self.pub_debug_array.publish(msg)




# -----------------------------------------------------------------------------
#  main
# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ImpedancePlanner()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    finally:
        exec.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
