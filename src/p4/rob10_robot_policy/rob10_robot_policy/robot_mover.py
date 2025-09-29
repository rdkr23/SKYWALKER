#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GoToJointPose(Node):
    JOINT_NAMES = [
        'joint1',
        'joint2',
        'joint3',
        'joint4',
        'joint5',
        'joint6',
        'joint7',
    ]
    CMD_TOPIC = '/xarm7_traj_controller/joint_trajectory'

    def __init__(self):
        super().__init__('go_to_joint_pose')

        self.pub = self.create_publisher(JointTrajectory, self.CMD_TOPIC, 100)

        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES  # Must be a list of strings

        point = JointTrajectoryPoint()
        point.positions = [0.01221730, -0.04014257, 0.01221730,
                           0.53929221, 0.01221730, -1.00356287, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)  # OK for ROS 2
        traj.points.append(point)

        self.pub.publish(traj)

def main():
    rclpy.init()
    node = GoToJointPose()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
