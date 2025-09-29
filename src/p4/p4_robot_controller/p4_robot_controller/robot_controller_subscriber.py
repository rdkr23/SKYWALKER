#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RobotControllerSubscriber(Node):
    def __init__(self):
        super().__init__('robot_controller_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/xarm/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received joint states: "%s"' % msg.position)

def main(args=None):
    rclpy.init(args=args)
    rc_subscriber = RobotControllerSubscriber()
    rclpy.spin(rc_subscriber)
    rc_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()