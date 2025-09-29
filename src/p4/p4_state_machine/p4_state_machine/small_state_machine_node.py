import time
import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, Int16

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.get_logger().info("Initializing StateMachine")

        self.node_runner = None

        self.tag_number = 64

        self.state = 12

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=100  # Used as a fallback for RMWs that require depth
        )

        self.rl_p2p_publisher = self.create_publisher(Int16, '/rl/activate_p2p', qos_profile)
        self.rl_p2p_state = self.create_subscription(Bool, '/rl/response_p2p', self.rl_p2p_callback, 10)

        self.rl_servo_align = self.create_publisher(Int16, '/rl/activate_servo_align', 10)

        data = Int16()
        data.data = self.tag_number
        self.get_logger().info('Calling RL P2P Publisher')
        self.rl_p2p_publisher.publish(data)

    def rl_p2p_callback(self, msg):
        self.get_logger().info('Done with rl p2p')
        data = Int16()
        data.data = self.tag_number

        self.get_logger().info("Beginning servo align")
        self.rl_servo_align.publish(data)



def main():
    rclpy.init()
    state_machine = StateMachine()
    rclpy.spin(state_machine)


if __name__ == '__main__':
    main()
