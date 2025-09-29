import rclpy
import time
import numpy as np
from numpy.linalg import inv
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from p4_robot_controller.servo_robot_controller import ServoPlanner
from p4_robot_controller.transformation_functions import quad_to_tm, tm_to_quad, quat_to_rpy, rpy_to_quat
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from xarm_msgs.srv import SetInt16


class ServoVisionController:
    def __init__(self, node):
        self.node = node

        self.servo = ServoPlanner(self.node)
        self.servo.initiate()

        self.camera_offset_crd = [0, 0, 0.2, 1, 0, 0, 0]# 0.7071068, -0.7071068, 0, 0

        self.k = 0.1 # Regulator value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.activated = False
        self.in_front = False

        self.aligned = False

        self.goal_pos = [0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0]

        self.aligner = self.node.create_timer(0.1, self.run)

        self.tag_number = "0"

    def set_tag_number(self, number):
        self.tag_number = str(number)

    def create_frame(self):
        t = TransformStamped()

        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'link_base'
        t.child_frame_id = 'servo_goal_frame'

        t.transform.translation.x = self.goal_pos[0]
        t.transform.translation.y = self.goal_pos[1]
        t.transform.translation.z = self.goal_pos[2]
        t.transform.rotation.x = self.goal_pos[3]
        t.transform.rotation.y = self.goal_pos[4]
        t.transform.rotation.z = self.goal_pos[5]
        t.transform.rotation.w = self.goal_pos[6]

        self.tf_broadcaster.sendTransform(t)

    def activate(self):
        self.in_front = False
        self.aligned = False
        self.camera_offset_crd = [0, 0, 0.2, 1, 0, 0, 0]
        self.k = 0.1

        req = SetInt16.Request()
        req.data = 0

        client = self.node.create_client(SetInt16, "/xarm/set_state")

        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'service not available /xarm/set_state, waiting again...')

        client.call_async(req)

        # req = SetInt16.Request()
        # req.data = 1
        #
        # client = self.node.create_client(SetInt16, "/xarm/set_mode")
        #
        # while not client.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info(f'service not available /xarm/set_mode, waiting again...')
        #
        # client.call_async(req)

        self.activated = True

    def deactivate(self):
        self.aligned = False
        self.activated = False

    def run(self):
        # self.node.get_logger().info('Test')
        if self.activated:
            # self.node.get_logger().info('Running servo vision...')
            try:
                transform = self.tf_buffer.lookup_transform('link_eef',f'eef-tag36h11:{self.tag_number}', rclpy.time.Time()) #camera_color_optical_frame

                at_translation = transform.transform.translation
                at_rotation = transform.transform.rotation

                ap = [at_translation.x, at_translation.y, at_translation.z, at_rotation.x, at_rotation.y, at_rotation.z,
                        at_rotation.w]

                # ap = [at_translation.x, at_translation.y, at_translation.z, 0, 0, 0, 1]

                cp = self.servo.get_current_pose()
                tm_base_camera = quad_to_tm(cp)

                tm_camera_tag = quad_to_tm(ap)

                tm_tag_offset = quad_to_tm(self.camera_offset_crd)

                tm_base_tag = np.dot(tm_base_camera, np.dot(tm_camera_tag, tm_tag_offset))

                base_april_pose = tm_to_quad(tm_base_tag)

                ap_error = np.array(base_april_pose) - np.array(cp)

                ap_error_crd = np.array(base_april_pose[0:3]) - np.array(cp[0:3])
                ap_error_rot = np.array(quat_to_rpy(base_april_pose[3:7])) - np.array(quat_to_rpy(cp[3:7]))

                ap_error_rot[ap_error_rot > np.pi] -= 2 * np.pi
                ap_error_rot[ap_error_rot < -np.pi] += 2 * np.pi

                robot_crd = np.array(cp[0:3]) + ap_error_crd * self.k
                robot_rot = rpy_to_quat(np.array(quat_to_rpy(cp[3:7])) + ap_error_rot * self.k)

                robot_pos = np.concatenate((robot_crd, robot_rot))

                # robot_pos = np.array(cp) + ap_error * self.k
                robot_pos = robot_pos.tolist()

                if np.linalg.norm(ap_error[0:3]) < 0.005 and np.linalg.norm(ap_error[3:6]) < 0.01:
                    if not self.in_front:
                        self.in_front = True
                        self.k = 0.05
                        self.node.get_logger().info(f'In front')
                        self.camera_offset_crd = [0, 0.0, 0.076, 1, 0, 0, 0]

                    else:
                        self.aligned = True
                        self.node.get_logger().info(f'Aligned: {robot_pos}')
                        return
                else:
                    self.aligned = False


                self.goal_pos = robot_pos

                self.create_frame()

                self.servo.set_final_pose(robot_pos)
                self.servo.run()
            except Exception as e:
                self.node.get_logger().error(f'Could not get transforms: {e}')


