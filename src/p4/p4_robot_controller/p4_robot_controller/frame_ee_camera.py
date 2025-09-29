from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameEECamera(Node):

    def __init__(self):
        super().__init__('fixed_frame_tf2_ee_camera')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.03, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_eef'
        t.child_frame_id = 'camera_eef_color_optical_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0#0.5 #0.7071068
        t.transform.rotation.y = 0.0#-0.5 #0.0
        t.transform.rotation.z = 0.707#0.5 #0.0
        t.transform.rotation.w = 0.707#0.5 #0.7071068

        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_base'
        t.child_frame_id = 'camera_base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = -0.328
        t.transform.translation.z = 0.165+0.120+0.02
        t.transform.rotation.x = 0.0  # 0.7071068
        t.transform.rotation.y = -0.087  # 0.0
        t.transform.rotation.z = 0.0  # 0.0
        t.transform.rotation.w = 0.996  # 0.7071068

        self.rotated_frames()

        self.tf_broadcaster.sendTransform(t)

    def rotated_frames(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_eef'
        t.child_frame_id = 'eef_rotated'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 0.707
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.707

        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_base'
        t.child_frame_id = 'base_rotated'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameEECamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()