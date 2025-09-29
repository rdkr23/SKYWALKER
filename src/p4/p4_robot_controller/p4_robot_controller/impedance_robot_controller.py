from p4_robot_controller.servo_robot_controller import ServoPlanner
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import rclpy
from p4_robot_controller.transformation_functions import (
    quad_to_tm,
    tm_to_quad,
    rpy_to_tm,
    tm_to_rpy,
    rpy_to_quat,
    euler_rpy_to_quat,
    quat_to_rpy,
)
from xarm_msgs.srv import SetInt16, FtForcePid, Call, FtCaliLoad, FtForceConfig, FtImpedance
from rclpy.qos import QoSProfile, ReliabilityPolicy
from xarm_msgs.msg import RobotMsg



def stamp_to_sec(st):
    """ROS2 builtin_interfaces/Time → seconds as float."""
    return st.sec + st.nanosec * 1e-9


class ImpedancePlanner:
    """
    Full mass‑spring‑damper (admittance) in Cartesian space.
    """

    def __init__(self, node):
        self.node = node

        # ── parameters ────────────────────────────────────────────────────────
        self.node.declare_parameter("mass", [2.0, 2.0, 2.0, 0.1, 0.1, 0.1])
        self.node.declare_parameter("stiffness", [100.0, 100.0, 100.0, 30.6, 30.6, 30.6])
        self.node.declare_parameter("damping", [32.0, 32.0, 32.0, 3.2, 3.2, 3.2])

        self.M = np.asarray(self.node.get_parameter("mass").value, float)
        self.B = np.asarray(self.node.get_parameter("damping").value, float)
        self.K = np.asarray(self.node.get_parameter("stiffness").value, float)

        # ── state ─────────────────────────────────────────────────────────────
        self.force_mask = None
        self.torque_mask = None
        self.set_mask([1, 1, 1, 1, 1, 1])

        self.force_filtered = np.zeros(3)
        self.torque_filtered = np.zeros(3)

        self.last_force = np.zeros(3)
        self.last_torque = np.zeros(3)

        self.goal_pose = None  

        self.servo = ServoPlanner(self.node)
        self.servo.initiate()

        self.activated = False

        self.current_crd = np.zeros(3)
        self.current_crd_dot = np.zeros(3)

        self.current_angle_delta = np.zeros(3)
        self.current_angle_dot = np.zeros(3)

        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.prev_time = None

        self.last_pose = None  # [x y z R P Y]
        self.last_robot_pose = np.zeros(3)
        self.pose = None  # [x y z qx qy qz qw]
        self.time = None

        # ── subscribers / publishers ─────────────────────────────────────────
        self.sub_force = self.node.create_subscription(
            WrenchStamped, "/xarm/uf_ftsensor_ext_states", self.force_callback, qos_sensor
        )

        self.pup_filtered_twist = self.node.create_publisher(
            WrenchStamped, "/xarm/filtered_force", 10
        )

        # self.sub_pose = self.node.create_subscription(
        #     RobotMsg, "/xarm/robot_states", self.cb_robot_pose, qos_sensor
        # )

        # control loop timer (100 hz)
        self.get_pose = self.node.create_timer(0.01, self.cb_robot_pose_tf)
        self.run_timer = self.node.create_timer(0.01, self.update)

        self.node.get_logger().info("FullImpedanceNode started with M, B, K …")

    # ─────────────────────────────────────────────────────────────────── set‑up
    def set_mask(self, mask):
        self.force_mask = mask[0:3]
        self.torque_mask = mask[3:6]

    # busy‑wait helper (unchanged)
    def get_current_pose(self):
        while True:
            pose = self.pose
            if isinstance(pose, np.ndarray) and pose.shape == (7,):
                return pose

    def activate(self):
        req = SetInt16.Request()
        req.data = 0
        client = self.node.create_client(SetInt16, "/xarm/set_state")
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("service not available /xarm/set_state …")
        client.call_async(req)

        # wait until we have a robot pose
        while True:
            pose = self.pose
            if isinstance(pose, np.ndarray) and pose.shape == (7,):
                break

        self.goal_pose = np.concatenate((pose[0:3], quat_to_rpy(pose[3:7])))

        self.current_crd.fill(0.0)
        self.current_crd_dot.fill(0.0)

        self.current_angle_delta.fill(0.0)
        self.current_angle_dot.fill(0.0)

        self.force_filtered.fill(0.0)
        self.torque_filtered.fill(0.0)

        self.prev_time = self.time

        self.activated = True
        self.node.get_logger().info("Impedance activated")

    def set_goal_pose(self, pose):
        self.goal_pose = np.asarray(pose)

    def deactivate(self):
        self.activated = False

    def service_call(self, service, variable, request):
        client = self.node.create_client(variable, service)

        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'service not available {service}, waiting again...')

        client.call_async(request)

    def force_callback(self, msg: WrenchStamped):
        self.time = stamp_to_sec(msg.header.stamp)
        self.last_force = np.array(
            [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        ) * self.force_mask
        self.last_torque = np.array(
            [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        ) * self.torque_mask

        self.last_force[np.abs(self.last_force) < 0.5] = 0.0
        self.last_torque[np.abs(self.last_torque) < 0.05] = 0.0

    def cb_robot_pose(self, msg: RobotMsg):
        self.last_pose = np.asarray(msg.pose, dtype=float)  
        self.last_robot_pose = self.last_pose[0:3] / 1000.0  # → metres
        self.last_robot_quat = rpy_to_quat(self.last_pose[3:6])
        self.pose = np.concatenate((self.last_robot_pose, self.last_robot_quat))

    def cb_robot_pose_tf(self):
        self.pose = np.asarray(self.servo.get_current_pose(), dtype=float)
        if isinstance(self.pose, np.ndarray) and self.pose.shape == (7,):
            self.last_robot_pose = self.pose[0:3]
            self.last_robot_quat = self.pose[3:7]
            self.last_pose = np.concatenate((self.last_robot_pose, quat_to_rpy(self.last_robot_quat)))
        else:
            self.pose = None

    # ───────────────────────────────────────────────────────── main control loop
    def update(self):
        if not self.activated or self.last_pose is None or self.time is None:
            return  # not ready yet

        # dt with glitch rejection ------------------------------------------------
        if self.prev_time is None:
            self.prev_time = self.time
            return
        dt = self.time - self.prev_time
        if dt <= 0.0 or dt > 0.05:
            dt = 0.01  # fall back to nominal 100Hz
        self.prev_time = self.time

        # rotation of current robot pose -----------------------------------------
        quat = rpy_to_quat(self.last_pose[3:6])
        tm = quad_to_tm([0, 0, 0, *quat])
        Rm = tm[:3, :3]

        # wrench to base, simple 1st‑order LPF ----------------------------------
        force_b = Rm @ self.last_force
        torque_b = self.last_torque  # assume sensor already gives base frame torques

        alpha = 0.001
        self.force_filtered = alpha * force_b + (1 - alpha) * self.force_filtered
        self.torque_filtered = (alpha * torque_b + (1 - alpha) * self.torque_filtered)

        # publish filtered wrench (optional)
        msg = WrenchStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = self.force_filtered
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = self.torque_filtered
        self.pup_filtered_twist.publish(msg)

        offset_linear = self.current_crd  # ← x(t)      internal state, starts at 0
        current_crd_ddot = (self.force_filtered - self.B[0:3] * self.current_crd_dot - self.K[0:3] * offset_linear) / self.M[0:3]

        self.current_crd_dot += current_crd_ddot * dt
        self.current_crd     += self.current_crd_dot * dt   # integrate state
        cmd_xyz = self.goal_pose[0:3] + self.current_crd

        # ── rotational admittance (still relative, keep old scheme) -------------
        offset_angular = self.current_angle_delta
        current_angle_ddot = (self.torque_filtered - self.B[3:6] * self.current_angle_dot - self.K[3:6] * offset_angular) / self.M[3:6]

        self.current_angle_dot += current_angle_ddot * dt
        self.current_angle_delta += self.current_angle_dot * dt

        tm_goal_R = rpy_to_tm([0, 0, 0, *self.goal_pose[3:6]])
        tm_total = tm_goal_R @ rpy_to_tm([0, 0, 0, *self.current_angle_delta])
        current_angle = np.asarray(tm_to_rpy(tm_total))[3:6]

        # ── send to low‑level servo --------------------------------------------
        pose = np.concatenate((cmd_xyz, rpy_to_quat(current_angle)))
        self.servo.set_final_pose(pose.tolist())
        self.servo.run()

class ImpedanceController:
    def __init__(self, node):
        self.node = node
        self.impedance_planner = ImpedancePlanner(self.node)

        self.linear_interp_freq = 50
        self.pose_with_time = []
        self.current_path = None
        self.path_executed = True
        self.node.create_timer(0.01, self._path_executor)

    def set_mask(self, mask):
        self.impedance_planner.set_mask(mask)

    def get_goal_pose(self):
        return self.impedance_planner.goal_pose

    def set_goal_pose(self, pose):
        self.impedance_planner.set_goal_pose(pose)

    def activate(self):
        self.impedance_planner.activate()

    def activated(self):
        if self.impedance_planner.activated:
            return True
        return False

    def deactivate(self):
        self.impedance_planner.deactivate()

    def _path_planner(self, pose, movement_time):
        self.path_executed = False
        t0 = time.time()

        start_pose = self.impedance_planner.goal_pose.tolist()
        coeffs = []  # a0, a1, a2, a3 per DOF
        for i in range(6):
            a0 = start_pose[i]
            a1 = 0.0
            a2 = 3.0 * (pose[i] - start_pose[i]) / movement_time ** 2
            a3 = -2.0 * (pose[i] - start_pose[i]) / movement_time ** 3
            coeffs.append([a0, a1, a2, a3])

        steps = int(movement_time * self.linear_interp_freq)
        for k in range(steps + 1):  # include t=0
            t = k / self.linear_interp_freq
            waypoint = [
                c[0] + c[1] * t + c[2] * t ** 2 + c[3] * t ** 3 for c in coeffs
            ]
            waypoint.append(t0 + t)
            self.pose_with_time.append(waypoint)

    def _path_executor(self):
        if not self.pose_with_time:
            return
        if self.current_path is None or self.current_path[6] < time.time():
            self.current_path = self.pose_with_time.pop(0)
            self.set_goal_pose(self.current_path[0:6])
            if not self.pose_with_time:
                self.path_executed = True

    def get_distance_difference(self):
        current_pose = self.impedance_planner.pose
        current_pose_rpy = np.concatenate((current_pose[0:3], quat_to_rpy(current_pose[3:])))

        return np.linalg.norm(current_pose_rpy - np.asarray(self.current_path[0:6]))

    def move_cartesian_path(self, pose, movement_time):
        self._path_planner(pose, movement_time)

    def move_cartesian_path_ee_frame(self, pose, movement_time):
        tm_start = rpy_to_tm(self.impedance_planner.goal_pose)
        tm_delta = rpy_to_tm(pose)
        tm_target = tm_start @ tm_delta
        new_pose = tm_to_rpy(tm_target)

        self._path_planner(new_pose, movement_time)
