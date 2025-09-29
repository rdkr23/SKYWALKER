import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int16
from smbus2 import SMBus
import time

class EndEffector(Node):
    def __init__(self):
        super().__init__('end_effector_node')
        try:
            self.bus = SMBus(1)
        except:
            self.get_logger().error("Could not start I2C bus")
            self.bus = None

        self.end_effector_subscriber = self.create_subscription(Int16, '/p4_state_machine/end_effector_controller',
                                                               self.run, 10)
        self.end_effector_state = self.create_publisher(Bool, '/p4_state_machine/end_effector_state', 10)

        self.address = 0x2b
        self.get_logger().info('Arduino Node Initialized')

    def send_data(self, data):
        if not self.bus:  # Check if serial connection exists
            self.get_logger().error("I2C Bus not available")
            return

        self.bus.write_byte_data(self.address, 0, data)
        time.sleep(0.05)
        start_time = time.time()
        while True:
            time.sleep(1)
            response = self.bus.read_byte_data(self.address, 0)
            if response == 1:
                self.get_logger().info(f"End Effector Node: Arduino response: {response}")
                self.end_effector_state.publish(Bool(data=True))
                break

            else:
                self.get_logger().info(f"End Effector Node: Arduino response: {response}")

    def run(self, msg):
        self.send_data(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
