import time
import numpy as np


from moveit.planning import MoveItPy
from rclpy.logging import get_logger
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose

import time
import rclpy
import numpy as np
from std_msgs.msg import Bool
from p4_robot_controller.impedance_robot_controller import ImpedanceController, ImpedancePlanner
from xarm_msgs.srv import SetInt16


class CrawlingMotion(Node):
    def __init__(self):
        super().__init__('docking_node')
        self.impedance_controller = ImpedanceController(self)
        self.impedance_controller.set_mask([0, 0, 0, 0, 0, 0])

        self.crawling_motion_subscriber = self.create_subscription(Bool, '/p4_state_machine/crawling_motion_controller', self.crawling_motion_callback, 10)
        self.crawling_motion_state = self.create_publisher(Bool, '/p4_state_machine/crawling_motion_state', 10)

        self.node_runner = None
        self.init_pose = None

        self.get_logger().info('Crawling Controller initialized')

    def crawling_motion_callback(self, msg):
        if msg.data:
            self.get_logger().info('Crawling State: Received callback')
            self.impedance_controller.activate()
            self.init_movement()

    def run(self):
        if self.impedance_controller.path_executed:
            self.node_runner.destroy()
            self.node_runner = None
            self.get_logger().info('Crawling: done')
            resp = Bool()
            resp.data = True
            self.crawling_motion_state.publish(resp)

            self.impedance_controller.deactivate()

    def init_movement(self):
        # self.get_logger().info('Docking State: docking')
        self.impedance_controller.move_cartesian_path_ee_frame([-0.4, 0, 0, 0, 0, 0], 15)
        self.node_runner = self.create_timer(0.01, self.run)

        self.get_logger().info('Crawling: initiated')


def main():
    rclpy.init()
    node = CrawlingMotion()
    rclpy.spin(node)

if __name__ == '__main__':
    main()