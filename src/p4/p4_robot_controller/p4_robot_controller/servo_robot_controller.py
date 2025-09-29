import time
import rclpy
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import ServoCommandType

from tf2_ros import TransformListener, Buffer
from p4_robot_controller.inverse_kin_class import InverseKinematicsCalculator

from control_msgs.msg import JointJog


class ServoPlanner:
    def __init__(self, node, base_frame_name="link_base", ee_frame_name = "link_eef", speed=0.1):
        self.node = node

        self.base_frame_name = base_frame_name
        self.ee_frame_name = ee_frame_name
        self.speed = speed

        self.pose_publisher = self.node.create_publisher(
            PoseStamped, "/servo_node/pose_target_cmds", 10
        )

        self.joint_publisher = self.node.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.servo_node_command_client = self.node.create_client(ServoCommandType, '/servo_node/switch_command_type')

        while not self.servo_node_command_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again /servo_node/switch_command_type...')

        self.current_pose = None

        self.pose_timer = self.node.create_timer(0.01, self._get_current_pose)
        self.final_pose = [0, 0, 0, 0, 0, 0, 1]
        self.inv_kin_calc = InverseKinematicsCalculator(self.node)
        self.command_type = 2


    def _get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame_name, self.ee_frame_name, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            self.current_pose = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z,
                                     rotation.w]
            # self.node.get_logger().info(f'Getting current pose {self.current_pose}')
        except Exception as e:
            self.node.get_logger().error(f'Could not get transform: {e} Trying again...')

    def get_current_pose(self):
        return self.current_pose

    def initiate(self, command_type=2):
        self.command_type = command_type
        cmd_type = ServoCommandType.Request()
        cmd_type.command_type = command_type

        future = self.servo_node_command_client.call_async(cmd_type)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info('Service established')


    def set_final_pose(self, pose):
        self.final_pose = pose
        # self.node.get_logger().info('Servo pose established')

    def run(self):
        try:
            if self.command_type == 2:
                pose_goal = PoseStamped()
                pose_goal.header.frame_id = self.base_frame_name #"link_base"

                pose_goal.pose.position.x = self.final_pose[0]
                pose_goal.pose.position.y = self.final_pose[1]
                pose_goal.pose.position.z = self.final_pose[2]
                pose_goal.pose.orientation.x = self.final_pose[3]
                pose_goal.pose.orientation.y = self.final_pose[4]
                pose_goal.pose.orientation.z = self.final_pose[5]
                pose_goal.pose.orientation.w = self.final_pose[6]

                pose_goal.header.stamp = self.node.get_clock().now().to_msg()
                self.pose_publisher.publish(pose_goal)

            # if self.command_type == 0:
            #     joints = self.inv_kin_calc.calc_inv_kin(self.final_pose)
            #
            #     joint_jog = JointJog()
            #     joint_jog.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
            #     joint_jog.displacements = joints
            #     joint_jog.header.stamp = self.node.get_clock().now().to_msg()
            #
            #     self.joint_publisher.publish(joint_jog)



        except Exception as e:
            self.node.get_logger().error(e)