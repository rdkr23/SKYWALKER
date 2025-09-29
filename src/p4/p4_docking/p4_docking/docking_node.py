import time
import numpy as np


from moveit.planning import MoveItPy
from rclpy.logging import get_logger
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose

import time
import rclpy
import numpy as np
from std_msgs.msg import Bool, Int16
from p4_robot_controller.impedance_robot_controller import ImpedanceController, ImpedancePlanner
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Docking(Node):
    def __init__(self):
        super().__init__('docking_node')
        self.impedance_controller = ImpedanceController(self)
        self.impedance_controller.set_mask([1, 1, 1, 1, 1, 1])
        # self.impedance_controller = ImpedancePlanner(self)
        # self.impedance_controller.set_mask([1, 1, 1, 1, 1, 1])
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.docking_subscriber = self.create_subscription(Int16, '/p4_state_machine/docking_controller', self.docking_callback, qos_profile)
        self.docking_state = self.create_publisher(Bool, '/p4_state_machine/docking_state', 10)

        self.node_runner = None

        self.init_pose = None

        self.get_logger().info('Docking Controller initialized')

    def docking_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info('Docking State: Received callback')
            self.impedance_controller.activate()
            self.get_logger().info('Docking State: docking')
            self.impedance_controller.move_cartesian_path_ee_frame([0, 0, 0.085, 0, 0, 0], 15)
            self.node_runner = self.create_timer(0.01, self.run)

            self.get_logger().info('Docking State: initiated')

        elif msg.data == 2:
            self.get_logger().info('Docking State: Received callback')
            self.impedance_controller.activate()
            self.get_logger().info('Docking State: undocking')
            self.impedance_controller.move_cartesian_path_ee_frame([0, 0, -0.1, 0, 0, 0], 10)
            self.node_runner = self.create_timer(0.01, self.run)

            self.get_logger().info('Docking State: initiated')

        else:
            self.get_logger().info(f'Docking State: FAILED {msg.data}')

    def run(self):
        if self.impedance_controller.path_executed:
            self.get_logger().info(f'Docking State: done {self.impedance_controller.get_distance_difference()}')
            self.node_runner.destroy()
            self.node_runner = None

            if self.impedance_controller.get_distance_difference() < 0.015:
                self.get_logger().info('Docking State: done')
                resp = Bool()
                resp.data = True
                self.docking_state.publish(resp)
                self.impedance_controller.deactivate()

            else:
                self.get_logger().info('Docking State: ERROR')
                resp = Bool()
                resp.data = False
                self.docking_state.publish(resp)
                self.impedance_controller.deactivate()


def main():
    rclpy.init()
    node = Docking()
    rclpy.spin(node)

if __name__ == '__main__':
    main()