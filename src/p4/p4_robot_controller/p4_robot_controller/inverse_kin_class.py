import PyKDL
from urdf_parser_py.urdf import URDF
from p4_robot_controller.b import treeFromUrdfModel
import math
from typing import Optional
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class InverseKinematicsCalculator:
    def __init__(self, node, urdf_path="src/p4/p4_robot_controller/p4_robot_controller/robot.urdf", base_link="link_base", eef_link="link7"):
        self.node = node

        robot = URDF.from_xml_file(urdf_path)
        ok, kdl_tree = treeFromUrdfModel(robot)

        if not ok:
            raise RuntimeError("KDL tree build failed: Could not find tree file.")

        self.base_link = "link_base"
        self.ee_link = "link7"
        self.chain = kdl_tree.getChain(self.base_link, self.ee_link)
        self.num_joints = self.chain.getNrOfJoints()

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

        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.node.create_subscription(JointState, "/xarm/joint_states", self.joint_state_cb, qos_sensor)


    def joint_state_cb(self, msg: JointState):
        if len(msg.position) < self.num_joints:
            return  #
        self.current_q = np.asarray(msg.position[: self.num_joints], dtype=float)

    def calc_inv_kin(self, quat):
        if self.current_q is None:
            return None

        x = quat[0]
        y = quat[1]
        z = quat[2]
        rx = quat[3]
        ry = quat[4]
        rz = quat[5]
        rw = quat[6]

        rot = PyKDL.Rotation.Quaternion(rx, ry, rz, rw)
        pos = PyKDL.Vector(x, y, z)
        target_frame = PyKDL.Frame(rot, pos)

        q_sol = self.compute_ik(target_frame, self.current_q)
        if q_sol is not None:
            return q_sol

        return None

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