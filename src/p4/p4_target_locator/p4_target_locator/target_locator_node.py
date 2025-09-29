import time
import numpy as np


from moveit.planning import MoveItPy
from rclpy.logging import get_logger
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose

from tf2_ros import TransformListener, Buffer, TransformBroadcaster

import time
import rclpy
import numpy as np
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import TransformStamped
from p4_robot_controller.impedance_robot_controller import ImpedanceController, ImpedancePlanner
from p4_robot_controller.transformation_functions import quad_to_tm, tm_to_quad, rpy_to_tm, tm_to_rpy, rpy_to_quat, euler_rpy_to_quat, quat_to_rpy


def main():
    rclpy.init()
    node = TargetLocator()
    rclpy.spin(node)


class TargetLocator(Node):
    def __init__(self):
        super().__init__('target_locator_node')
        self.impedance_controller = ImpedanceController(self)
        self.impedance_controller.set_mask([1, 1, 1, 0, 0, 0])

        self.camera_offset_crd = [0, 0, 0.5, 0.7071068, -0.7071068, 0, 0]

        self.tag_number_subscriber = self.create_subscription(Int16, '/p4_state_machine/tag_number', self.tag_number_callback, 10)

        self.target_locator_subscriber = self.create_subscription(Bool, '/p4_state_machine/target_locator_controller', self.target_locator_callback, 10)
        self.target_locator_state = self.create_publisher(Bool, '/p4_state_machine/target_locator_state', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.tag_number = None

        self.node_runner = None

        self.init_pose = None

        self.tag_pose = None

        self.get_tag_pose_timer = self.create_timer(0.1, self.get_tag_pose)

        self.get_logger().info('Target Locator: initialized')

    def tag_number_callback(self, msg):
        self.tag_number = str(msg.data)
        self.tag_pose = None

    def target_locator_callback(self, msg):
        if msg.data:
            self.init_movement()

    def create_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_base'
        t.child_frame_id = 'tag_pose_frame'

        t.transform.translation.x = self.tag_pose[0]
        t.transform.translation.y = self.tag_pose[1]
        t.transform.translation.z = self.tag_pose[2]
        t.transform.rotation.x = self.tag_pose[3]
        t.transform.rotation.y = self.tag_pose[4]
        t.transform.rotation.z = self.tag_pose[5]
        t.transform.rotation.w = self.tag_pose[6]

        self.tf_broadcaster.sendTransform(t)

    def run(self):
        if self.impedance_controller.path_executed:
            self.node_runner.destroy()
            self.node_runner = None
            self.get_logger().info('Target Locator: done')
            resp = Bool()
            resp.data = True
            self.impedance_controller.deactivate()
            self.target_locator_state.publish(resp)

    def get_tag_pose(self):
        try:
            if self.tag_number is None:
                return

            transform = self.tf_buffer.lookup_transform('link_base', f'base-tag36h11:{self.tag_number}',
                                                        rclpy.time.Time())

            at_translation = transform.transform.translation
            at_rotation = transform.transform.rotation

            ap = [at_translation.x, at_translation.y, at_translation.z, at_rotation.x, at_rotation.y, at_rotation.z,
                  at_rotation.w]

            tm_base_tag = quad_to_tm(ap)

            tm_tag_offset = quad_to_tm(self.camera_offset_crd)

            tm_base_tag = np.dot(tm_base_tag, tm_tag_offset)

            self.tag_pose = tm_to_quad(tm_base_tag)

            self.create_frame()

        except Exception as e:
            self.tag_pose = None
            if self.impedance_controller.activated():
                self.get_logger().error(f'Could not get transforms: {e}')

    def init_movement(self):
        if self.tag_pose is None:
            resp = Bool()
            resp.data = False
            self.get_logger().info(f'Target Locator: tag not found {self.tag_number}')
            self.target_locator_state.publish(resp)
            return

        tag_pose = self.tag_pose.copy()

        tag_pose = np.concatenate((tag_pose[0:3], quat_to_rpy(tag_pose[3:7])))

        self.impedance_controller.activate()
        self.impedance_controller.move_cartesian_path(tag_pose, 10)
        self.node_runner = self.create_timer(0.01, self.run)

        self.get_logger().info('Target Locator: initiated')

if __name__ == '__main__':
    main()