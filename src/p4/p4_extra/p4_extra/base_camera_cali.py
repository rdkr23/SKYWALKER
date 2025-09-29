# File: calibrated_camera_info_pub.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import yaml

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('calibrated_camera_info_publisher')

        # Load camera intrinsics from YAML file
        calib_path = 'src/p4/p4_extra/cfg/cali.yaml'
        with open(calib_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width  = calib_data['image_width']
        self.camera_info_msg.height = calib_data['image_height']
        self.camera_info_msg.distortion_model = calib_data['distortion_model']
        self.camera_info_msg.d = calib_data['distortion_coefficients']['data']
        self.camera_info_msg.k = calib_data['camera_matrix']['data']
        self.camera_info_msg.r = calib_data['rectification_matrix']['data']
        self.camera_info_msg.p = calib_data['projection_matrix']['data']
        self.camera_info_msg.binning_x = 0
        self.camera_info_msg.binning_y = 0

        self.publisher = self.create_publisher(CameraInfo, '/camera/color/camera_info_calibrated', 10)
        self.timer = self.create_timer(1/30.0, self.timer_callback)

    def timer_callback(self):
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_info_msg.header.frame_id = "camera_color_optical_frame"
        self.publisher.publish(self.camera_info_msg)

def main():
    rclpy.init()
    node = CameraInfoPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
