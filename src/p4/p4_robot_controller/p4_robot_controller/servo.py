#!/usr/bin/env python3
"""
Forward IK at 100 Hz to the robot via the xArm Python SDK.
"""

import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from xarm.wrapper import XArmAPI
from rclpy.qos import QoSProfile, ReliabilityPolicy

class IKStreamer(Node):
    def __init__(self, ip: str, rate_hz: int = 100):
        super().__init__('ik_streamer')
        self.arm = XArmAPI(ip)
        self.period = 1.0 / rate_hz
        self.last_send = time.time()

        self.arm.motion_enable(True)
        self.arm.set_mode(1)
        self.arm.set_state(0)
        time.sleep(0.1)
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/servo/ik_joint_solution',
            self.cb,
            qos_sensor)
        

    # ───────────────────────────────────────────
    def cb(self, msg: Float64MultiArray):
        now = time.time()
        if now - self.last_send < self.period:
            return                            

        if len(msg.data) != 7:
            self.get_logger().warn('Bad IK array len %d', len(msg.data))
            return

        self.arm.set_servo_angle_j(msg.data, is_radian=True, speed=0.01, acc=0.00)
        self.last_send = now


def main():
    rclpy.init()
    node = IKStreamer('192.168.1.198', rate_hz=250)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
