import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int16
from smbus2 import SMBus

class BrakeNode(Node):
    def __init__(self):
        super().__init__('brake_node')
        try:
            self.bus = SMBus(1)
        except:
            self.get_logger().error("Brake Node: Error in I2C bus: ")
            self.bus = None

        self.brake_subscriber = self.create_subscription(Int16, '/p4_state_machine/brake_controller', self.run, 10)
        self.brake_state = self.create_publisher(Bool, '/p4_state_machine/brake_state', 10)
        self.address = 0x2a
        self.get_logger().info('Brake Node: Arduino Node Initialized')

    def send_data(self, data):
        if not self.bus:
            self.get_logger().error("Brake Node: I2C Bus not available")
            return
        self.bus.write_byte_data(self.address, 0, data)
        time.sleep(0.05)

        while True:
            time.sleep(1)
            response = self.bus.read_byte_data(self.address, 0)
            if response == 1:
                self.get_logger().info(f"Brake Node: Arduino response: {response}")
                self.brake_state.publish(Bool(data=True))
                break

            else:
                self.get_logger().info(f"Brake Node: Arduino response: {response}")

    def run(self, msg):
        self.send_data(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = BrakeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

