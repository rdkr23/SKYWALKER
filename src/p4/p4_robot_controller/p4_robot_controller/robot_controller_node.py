import time
import numpy as np


from moveit.planning import MoveItPy
from rclpy.logging import get_logger
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose

import time
import rclpy
import numpy as np

from p4_robot_controller.servo_vision_alignment import ServoVisionController


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


class Main(Node):
    def __init__(self, robot, robot_arm):
        super().__init__('test_node')

        self.get_logger().info('Initializing Robot')
        self.robot = robot
        self.get_logger().info('Initialized Robot')
        self.robot_arm = robot_arm
        self.get_logger().info('Initialized Robot Controller Node')

        move_robot_helper(self.robot, self.robot_arm, [0.465, -0.041, 0.410, -0.7098926, 0.0120876, -0.7037825, 0.0244245])
        self.get_logger().info('Robot start')
        self.servo_vision = ServoVisionController(self)

    def run(self):
        while 1:
            rclpy.spin_once(self)
            self.servo_vision.run()


def main():
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")
    logger.info("Started")

    xarm7 = MoveItPy()

    xarm7_arm = xarm7.get_planning_component("xarm7")
    root = Main(xarm7, xarm7_arm)
    root.run()


    #
    # move_robot_helper(xarm7, xarm7_arm, [-0.28, 0.12, 0.5, 0, 0, 0, 1])
    #
    # pose_goal = PoseStamped()
    # pose_goal.header.frame_id = "link_base"
    # pose_goal.pose.orientation.w = 1.0
    # pose_goal.pose.position.x = -0.28
    # pose_goal.pose.position.y = -0.12
    # pose_goal.pose.position.z = 0.5
    # xarm7_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_eef")
    #
    # plan_result = xarm7_arm.plan()
    # robot_trajectory = plan_result.trajectory
    # xarm7.execute(robot_trajectory, controllers=[])

if __name__ == '__main__':
    main()