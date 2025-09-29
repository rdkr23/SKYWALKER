import time
import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, Int16

from p4_robot_controller.impedance_robot_controller import ImpedancePlanner, ImpedanceController
from xarm_msgs.srv import SetInt16, FtForcePid, Call, FtCaliLoad, FtForceConfig, FtImpedance


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.get_logger().info("Initializing StateMachine")


        self.node_runner = None

        self.tag_sequence = [0, 64]

        self.state = 12

        self.tag_number_publisher = self.create_publisher(Int16, '/p4_state_machine/tag_number', 10)

        self.target_locator_publisher = self.create_publisher(Bool, '/p4_state_machine/target_locator_controller', 10)
        self.target_locator_state = self.create_subscription(Bool, '/p4_state_machine/target_locator_state', self.target_locator_callback, 10)

        self.servo_align_publisher = self.create_publisher(Bool, '/p4_state_machine/servo_align_controller', 10)
        self.servo_align_state = self.create_subscription(Bool, '/p4_state_machine/servo_align_state', self.servo_align_callback, 10)

        self.docking_publisher = self.create_publisher(Int16, '/p4_state_machine/docking_controller', 10)
        self.docking_state = self.create_subscription(Bool, '/p4_state_machine/docking_state', self.docking_callback, 10)

        self.brake_publisher = self.create_publisher(Int16, '/p4_state_machine/brake_controller', 10)
        self.brake_state = self.create_subscription(Bool, '/p4_state_machine/brake_state', self.brake_controller_callback, 10)
        
        self.end_effector_publisher = self.create_publisher(Int16, '/p4_state_machine/end_effector_controller', 10)
        self.end_effector_state = self.create_subscription(Bool, '/p4_state_machine/end_effector_state', self.ee_controller_callback, 10)

        self.crawling_motion_publisher = self.create_publisher(Bool, '/p4_state_machine/crawling_motion_controller', 10)
        self.crawling_motion_state = self.create_subscription(Bool, '/p4_state_machine/crawling_motion_state', self.crawling_motion_callback, 10)

        time.sleep(1)

        self.init_force_torque_sensor()

        time.sleep(2)
        self.run()

    def init_force_torque_sensor(self):
        req = FtCaliLoad.Request()
        req.datas = [1.170437216758728, 6.8034796714782715, 33.474945068359375, 56.690589904785156,
                     34.510169982910156, 32.048709869384766, 304.669921875, 3.4369821548461914,
                     1.7179324626922607, -0.882534384727478]
        req.association_setting_tcp_load = True
        req.m = 1.170437216758728
        req.x = 6.8034796714782715
        req.y = 33.474945068359375
        req.z = 56.690589904785156
        self.service_call("/xarm/ft_sensor_cali_load", FtCaliLoad, req)

        req = SetInt16.Request()
        req.data = 1
        self.service_call("/xarm/ft_sensor_enable", SetInt16, req)

        req = Call.Request()
        self.service_call("/xarm/ft_sensor_set_zero", Call, req)
        #
        # req = SetInt16.Request()
        # req.data = 1
        # self.service_call("/xarm/ft_sensor_app_set", SetInt16, req)

    def service_call(self, service, variable, request):
        client = self.create_client(variable, service)

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service not available {service}, waiting again...')

        client.call_async(request)

    def target_locator_callback(self, msg):
        if msg.data:
            self.get_logger().info("(STATE 10 -> STATE 11) Target locator completed")
            self.tag_sequence.pop(0)
            self.state = 11
            self.run()
        else:
            self.get_logger().info("(STATE 10 -> STATE 9) Target locator not completed")
            self.state = 9
            self.run()

    def servo_align_callback(self, msg):
        if msg.data:
            self.get_logger().info("(STATE 11 -> STATE 12) Servo alignment completed")
            self.state = 12
            self.run()

    def docking_callback(self, msg):
        if self.state == 12:
            if msg.data:
                self.get_logger().info("(STATE 12 -> STATE 13) Docking completed")
                self.state = 13
                self.run()
            else:
                self.get_logger().info("(STATE 12 -> STATE 11) Docking failed")
                self.state = 11
                self.run()

        elif self.state == 18:
            if msg.data:
                self.get_logger().info("(STATE 18 -> STATE 10) Undocking completed")
                self.state = 9
                self.run()
            else:
                self.get_logger().info("(STATE 18 -> STATE 18) Undocking failed")
                self.state = 18
                self.run()

    def ee_controller_callback(self, msg):
        if self.state == 13:
            if msg.data:
                self.get_logger().info("(STATE 13 -> STATE 14) End effector locked")
                self.state = 14
                self.run()

        elif self.state == 17:
            if msg.data:
                self.get_logger().info("(STATE 17 -> STATE 18) End effector unlocked")
                self.state = 18
                self.run()

        elif self.state == 1:
            if msg.data:
                self.get_logger().info("(STATE 1 -> STATE 2) End effector unlocked")
                self.state = 2
                self.run()

    def brake_controller_callback(self, msg):
        if self.state == 2:
            if msg.data:
                self.get_logger().info("(STATE 2 -> STATE 3) Brakes homed")
                self.state = 3
                self.run()

        elif self.state == 3:
            if msg.data:
                self.get_logger().info("(STATE 3 -> STATE 4) Brakes activated")
                self.state = 10
                self.run()

        elif self.state == 14:
            if msg.data:
                self.get_logger().info("(STATE 14 -> STATE 15) Brakes homed")
                self.state = 15
                self.run()

        elif self.state == 16:
            if msg.data:
                self.get_logger().info("(STATE 16 -> STATE 17) Brakes activated")
                self.state = 17
                self.run()

    def crawling_motion_callback(self, msg):
        if msg.data:
            self.get_logger().info("(STATE 15 -> STATE 16) Crawling motion completed")
            self.state = 16
            self.run()

    def home_position_callback(self):
        if self.impedance_controller.path_executed:
            self.node_runner.destroy()
            self.node_runner = None
            self.get_logger().info('(STATE 4 -> STATE 9) Moved to starting position')
            self.impedance_controller.deactivate()
            self.state = 9

    def run(self):
        self.get_logger().info(f"Current state: {self.state}")

        # Initializing state
        if self.state == 0:
            self.get_logger().info("(STATE 0) Initializing state machine")
            self.state = 1
            self.run()

        elif self.state == 1:
            resp = Int16()
            resp.data = 1
            self.end_effector_publisher.publish(resp)

            self.get_logger().info("(STATE 1) Homing end-effector")

        elif self.state == 2:
            resp = Int16()
            resp.data = 1
            self.brake_publisher.publish(resp)
            self.get_logger().info("(STATE 2) Homing brakes")

        elif self.state == 3:
            resp = Int16()
            resp.data = 2
            self.brake_publisher.publish(resp)
            self.get_logger().info("(STATE 3) Activating brakes")

        elif self.state == 4:
            self.get_logger().info("(STATE 4) Moving robot to start position.")
            self.impedance_controller.activate()
            self.impedance_controller.move_cartesian_path(np.array([-0.2428, -0.1791, 0.5335, -98.9/180*np.pi, 0, 93.1/180*np.pi]), 10)
            self.node_runner = self.create_timer(0.01, self.home_position_callback)

        elif self.state == 9:
            if len(self.tag_sequence) == 0:
                self.get_logger().info("(STATE 9) No tag number available. Expecting sequence is done.")
                return

            self.get_logger().info("(STATE 9) Received tag number and publishing it to nodes.")
            tag_no = Int16()
            tag_no.data = self.tag_sequence[0]
            self.tag_number_publisher.publish(tag_no)
            self.state = 10
            self.run()

        elif self.state == 10:
            resp = Bool()
            resp.data = True
            self.target_locator_publisher.publish(resp)
            self.get_logger().info("(STATE 10) Getting tag number and publishing it to nodes. Target locator started")

        # Servo alignment
        elif self.state == 11:
            resp = Bool()
            resp.data = True
            self.servo_align_publisher.publish(resp)
            self.get_logger().info("(STATE 11) Starting servo alignment.")
        # Docking state
        elif self.state == 12:
            resp = Int16()
            resp.data = 1
            self.docking_publisher.publish(resp)
            self.get_logger().info("(STATE 12) Starting docking.")

        # End-effector activation state
        elif self.state == 13:
            resp = Int16()
            resp.data = 2

            self.end_effector_publisher.publish(resp)
            self.get_logger().info("(STATE 13) End-effector started.")

        # Brake release state
        elif self.state == 14:
            resp = Int16()
            resp.data = 1

            self.brake_publisher.publish(resp)
            self.get_logger().info("(STATE 14) Releasing brakes.")

        # Movement state
        elif self.state == 15:
            time.sleep(5)
            resp = Bool()
            resp.data = True
            self.crawling_motion_publisher.publish(resp)
            self.get_logger().info("(STATE 15) Starting crawling motion.")

        elif self.state == 16:
            resp = Int16()
            resp.data = 2
            self.brake_publisher.publish(resp)
            self.get_logger().info("(STATE 16) Activating brakes")

        elif self.state == 17:
            resp = Int16()
            resp.data = 1
            self.end_effector_publisher.publish(resp)
            self.get_logger().info("(STATE 17) Deactivating end-effector")

        elif self.state == 18:
            resp = Int16()
            resp.data = 2
            self.docking_publisher.publish(resp)
            self.get_logger().info("(STATE 18) Starting undocking.")

def main():
    rclpy.init()
    state_machine = StateMachine()
    rclpy.spin(state_machine)




if __name__ == '__main__':
    main()
