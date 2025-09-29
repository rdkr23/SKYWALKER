import time
import numpy as np


from moveit.planning import MoveItPy
from rclpy.logging import get_logger
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose

import time
import rclpy
import numpy as np

from p4_servo_vision.servo_vision_alignment import ServoVisionController
from std_msgs.msg import Bool, Int16


def move_robot_helper(robot, robot_arm, pose):
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "link_base"
    pose_goal.pose.position.x = float(pose[0])
    pose_goal.pose.position.y = float(pose[1])
    pose_goal.pose.position.z = float(pose[2])

    pose_goal.pose.orientation.x = float(pose[3])
    pose_goal.pose.orientation.y = float(pose[4])
    pose_goal.pose.orientation.z = float(pose[5])
    pose_goal.pose.orientation.w = float(pose[6])

    robot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_eef")

    plan_result = robot_arm.plan()
    robot_trajectory = plan_result.trajectory
    robot.execute(robot_trajectory, controllers=[])


class ServoVision(Node):
    def __init__(self, robot, robot_arm):
        super().__init__('servo_vision_node')
        self.robot = robot
        self.robot_arm = robot_arm

        self.servo_vision = ServoVisionController(self)

        self.tag_number_subscriber = self.create_subscription(Int16, '/rl/activate_servo_align', self.tag_number_callback, 10)
        self.servo_align_state = self.create_publisher(Bool, '/rl/response_servo_align', 10)

        self.get_logger().info('Servo Vision Controller initialized')

        self.aligner = self.create_timer(0.1, self.servo_aligned_timer)

    def tag_number_callback(self, msg):
        self.servo_vision.set_tag_number(msg.data)
        self.run()

    def servo_aligned_timer(self):
        if self.servo_vision.aligned:
            self.get_logger().info('Aligned')
            self.servo_vision.deactivate()
            resp = Bool()
            resp.data = True
            self.servo_align_state.publish(resp)

    def run(self):
        self.servo_vision.activate()
        self.get_logger().info('Servo Vision Controller activated and running')


def main():
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")
    logger.info("Started")

    # xarm7 = MoveItPy()
    #
    # xarm7_arm = xarm7.get_planning_component("xarm7")

    xarm7 = None
    xarm7_arm = None
    node = ServoVision(xarm7, xarm7_arm)
    rclpy.spin(node)

if __name__ == '__main__':
    main()