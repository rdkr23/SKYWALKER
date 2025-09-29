import os
import sys
import yaml
import time
import rclpy
import numpy as np
from math import cos, sin
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
# message libraries
from geometry_msgs.msg import PoseStamped, Pose
import threading
from robot_controller.servo_robot_controller import ServoPlanner


from moveit.planning import MoveItPy
from rclpy.node import Node

from moveit.core.robot_state import RobotState

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

from copy import deepcopy
from typing import Optional, Tuple

from control_msgs.msg import JointJog

from moveit_msgs.srv import ServoCommandType

from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from std_srvs.srv import Trigger



def plan_and_execute(robot, planning_component, logger, single_plan_parameters=None, multi_plan_parameters=None, sleep_time=0.0,):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():
    rclpy.init()

    logger = get_logger("moveit_py.pose_goal")
    logger.info("Started")

    xarm7 = MoveItPy()

    xarm7_arm = xarm7.get_planning_component("xarm7")

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "link_base"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = -0.28
    pose_goal.pose.position.y = -0.12
    pose_goal.pose.position.z = 0.5
    xarm7_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_eef")

    plan_result = xarm7_arm.plan()
    robot_trajectory = plan_result.trajectory
    xarm7.execute(robot_trajectory, controllers=[])

    time.sleep(10)

    servo = ServoPlanner()
    servo.initiate()
    servo.set_final_pose([0.28, -0.12, 0.5, 0, 0, 0, 1])
    while 1:
        servo.execute()


if __name__ == "__main__":
    main()