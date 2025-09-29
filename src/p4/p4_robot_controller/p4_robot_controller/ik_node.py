#!/usr/bin/env python3
"""xArm IK 

• Listens to the impedance‑planner pose on `/xarm/impedance_offset`
• Runs KDL inverse kinematics for the 7‑DoF chain (link_base → link7)
• Publishes the joint‑angle solution on `/xarm/ik_joint_solution` (Float64[7])
  and can optionally call the `xarm/move_joint` service if you enable it.
"""

import math
from typing import Optional

import numpy as np
import PyKDL
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import URDF


from xarm_msgs.srv import MoveJoint
from p4_robot_controller.transformation_functions import  quat_to_rpy


from p4_robot_controller.b import treeFromUrdfModel  # helper that builds a KDL tree from URDF


class XarmKDLMonitor(Node):
    """Collects joint state, solves IK for a pose, and republishes joint angles."""

    def __init__(self):
        super().__init__("xarm_kdl_monitor")

        # ───────────────────────────────────
        robot = URDF.from_xml_file(
            "src/p4/p4_robot_controller/p4_robot_controller/robot.urdf"
        )
        ok, kdl_tree = treeFromUrdfModel(robot)
        if not ok:
            self.get_logger().fatal("Failed to build KDL tree from URDF")
            raise RuntimeError("KDL tree build failed")

        self.base_link = "link_base"
        self.ee_link = "link7"
        self.chain = kdl_tree.getChain(self.base_link, self.ee_link)
        self.num_joints = self.chain.getNrOfJoints()

        # ─────────────────────────────────────────────────────
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)

        self.jl_min = PyKDL.JntArray(self.num_joints)
        self.jl_max = PyKDL.JntArray(self.num_joints)
        real_deg_limits = [175, 110, 175, 170, 175, 110, 175]        
        for i, deg in enumerate(real_deg_limits):
            lim = math.radians(deg)
            self.jl_min[i] = -lim
            self.jl_max[i] =  lim

        self.ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(self.chain)
        self.ik_pos_solver = PyKDL.ChainIkSolverPos_NR_JL(             
                self.chain, self.jl_min, self.jl_max,
                self.fk_solver, self.ik_vel_solver,
                maxiter=100, eps=1e-6)

        # ───────────────────────────────────────────────────────
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(JointState, "/xarm/joint_states", self.joint_state_cb, qos_sensor)
        self.create_subscription(Float64MultiArray, "/xarm/debug_array", self.pose_cb, qos_sensor)

        self.pub_ik = self.create_publisher(Float64MultiArray, "/xarm/ik_joint_solution", qos_sensor)

        self.create_timer(0.001, self.timer_cb)  # 100 Hz
        # ────────────────────────────────────────────────────
        self.prev_time = None


        self.get_logger().info("xArm IK monitor ready – waiting for joint states…")
        self.current_q = None
        self.latest_pose = None



    # ─────────────────────────────────────────────────────────

    def joint_state_cb(self, msg: JointState):
        if len(msg.position) < self.num_joints:
            return  # 
        self.current_q = np.asarray(msg.position[: self.num_joints], dtype=float)



        # ─────────────────
    def pose_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            self.get_logger().warn("Pose array must have 6 elements")
            return
        self.latest_pose = msg

    def timer_cb(self):
        if self.current_q is None or self.latest_pose is None:
            return  # Wait until both are ready

        x, y, z, roll, pitch, yaw = self.latest_pose.data
        mm_to_m = 1e-3

        rot = PyKDL.Rotation.RPY(roll, pitch, yaw)
        pos = PyKDL.Vector(x * mm_to_m, y * mm_to_m, z * mm_to_m)
        target_frame = PyKDL.Frame(rot, pos)

        q_sol = self.compute_ik(target_frame, self.current_q)
        if q_sol is not None:
            self._pub_array(self.pub_ik, q_sol)



    # ────────────────────────────────────────────────────────────

    def compute_ik(self, target_frame: PyKDL.Frame, seed: np.ndarray) -> Optional[np.ndarray]:
        q_seed = PyKDL.JntArray(self.num_joints)
        for i, q in enumerate(seed):
            q_seed[i] = float(q)
        q_out = PyKDL.JntArray(self.num_joints)
        ret = self.ik_pos_solver.CartToJnt(q_seed, target_frame, q_out)
        if ret < 0:
            self.get_logger().error(f"KDL IK failed with code {ret}")
            return None
        return np.array([q_out[i] for i in range(self.num_joints)], dtype=float)

    def _pub_array(self, pub, data):
        msg = Float64MultiArray()
        msg.data = list(map(float, np.ravel(data)))
        pub.publish(msg)


# ────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = XarmKDLMonitor()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    finally:
        exec.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
