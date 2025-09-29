#!/usr/bin/env python3
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import URDF
import PyKDL

from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix

from p4_robot_controller.b import treeFromUrdfModel     # your local helper

class XarmKDLMonitor(Node):
    def __init__(self):
        super().__init__('xarm_kdl_monitor')
        
        pkg_share = get_package_share_directory('manual_rectify')
        urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
        robot     = URDF.from_xml_file(urdf_path)
        
        
        ok, kdl_tree = treeFromUrdfModel(robot)
        if not ok:
            self.get_logger().fatal('Failed to build KDL tree'); return

        self.base_link = 'link_base'
        self.ee_link   = 'link7'
        self.chain     = kdl_tree.getChain(self.base_link, self.ee_link)
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)
        self.num_joints = self.chain.getNrOfJoints()

        # ---------- TF2 buffer ----------
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- state cache ----------
        self.prev_pos  = None
        self.prev_time = None

        # ---------- pubs & subs ----------
        self.pub_cartesian_twist= self.create_publisher(Float64MultiArray, '/xarm/cartesian_velocity',10)


        # Forward kinematics solver
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        # Velocity IK (for NR solver)
        self.ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(self.chain)
        # Position IK (Newton–Raphson)
        self.ik_pos_solver = PyKDL.ChainIkSolverPos_NR(self.chain,
                                                    self.fk_solver,
                                                    self.ik_vel_solver,
                                                    maxiter=100,
                                                    eps=1e-6)


        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)


    # --------------------------------------------------
    def joint_state_cb(self, msg: JointState):
        cur_pos = np.asarray(msg.position[:self.num_joints], dtype=float)

        # first sample → initialise
        now = self.get_clock().now()
        if self.prev_pos is None:
            self.prev_pos  = cur_pos
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:                                      # guard against clock issues
            return

        qdot = (cur_pos - self.prev_pos) / dt
        self.prev_pos  = cur_pos.copy()
        self.prev_time = now

        # ---- KDL Jacobian ----
        q_kdl = PyKDL.JntArray(self.num_joints)
        for i, q_i in enumerate(cur_pos):
            q_kdl[i] = q_i
        J = PyKDL.Jacobian(self.num_joints)
        self.jac_solver.JntToJac(q_kdl, J)
        J_np = np.array([[J[i, j] for j in range(self.num_joints)] for i in range(6)])

        twist_base = J_np @ qdot         # [vx,vy,vz, wx,wy,wz] in base frame

       

        # ---- publish ----
        self._pub_array(self.pub_cartesian_twist, twist_base)

    # small helper
    def _pub_array(self, pub, data):
        msg = Float64MultiArray()
        msg.data = data.tolist()
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = XarmKDLMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
