import rclpy
import time
import numpy as np
from rclpy.node import Node
from xarm_msgs.srv import MoveCartesian
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from custom_msg.srv import GetAprilPos


class RobotControllerPublisher(Node):
    def __init__(self):
        super().__init__('robot_controller_publisher')

        self.robot_running = True

        self.robot_position = [462, 0, 711, 180 / 180 * 3.14, -90 / 180 * 3.14, 0]

        self.robot_state = self.create_subscription(JointState, '/xarm/joint_states', self.get_robot_state, 1)
        self.robot_state

        self.control_robot = self.create_client(MoveCartesian, '/xarm/set_position')
        self.april_tag_reader = self.create_client(GetAprilPos, 'april_tag')

        self.control_robot_request = MoveCartesian.Request()
        self.april_tag_reader_request = GetAprilPos.Request()

    def get_robot_state(self, msg):
        if all(velocity == 0 for velocity in msg.velocity):
            self.robot_running = False
        else:
            self.robot_running = True

    def read_april_tag(self, april_tag_id):
        self.april_tag_reader_request.april_tag_id = april_tag_id
        return self.april_tag_reader.call_async(self.april_tag_reader_request)

    def send_robot_pos(self, robot_pose):
        self.control_robot_request.pose = robot_pose
        return self.control_robot.call_async(self.control_robot_request)


def crd_to_tm(position):
    x = position[0]
    y = position[1]
    z = position[2]

    rx = position[3]
    ry = position[4]
    rz = position[5]

    tm = np.array([[1, 0, 0, x],
                   [0, 1, 0, y],
                   [0, 0, 1, z],
                   [0, 0, 0, 1]])

    rotx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(rx), -np.sin(rx), 0],
        [0, np.sin(rx), np.cos(rx), 0],
        [0, 0, 0, 1]
    ])

    roty = np.array([
        [np.cos(ry), 0, np.sin(ry), 0],
        [0, 1, 0, 0],
        [-np.sin(ry), 0, np.cos(ry), 0],
        [0, 0, 0, 1]
    ])

    rotz = np.array([
        [np.cos(rz), -np.sin(rz), 0, 0],
        [np.sin(rz), np.cos(rz), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    rm = np.dot(np.dot(rotz, roty), rotx)

    # Combine the translation and rotation matrices
    tm = np.dot(tm, rm)

    return tm


def tm_to_crd(tm):
    x = tm[0,3]
    y = tm[1,3]
    z = tm[2,3]

    rm = tm[0:3,0:3]

    sy = np.sqrt(rm[0, 0] ** 2 + rm[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        r = np.arctan2(rm[2, 1], rm[2, 2])
        p = np.arctan2(-rm[2, 0], sy)
        y = np.arctan2(rm[1, 0], rm[0, 0])
    else:
        r = np.arctan2(-rm[1, 2], rm[1, 1])
        p = np.arctan2(-rm[2, 0], sy)
        y = 0

    return [x, y, z, r, p, y]


def main(args=None):
    rclpy.init(args=args)

    camera_dist = [0,0,50,0,0,0]
    k = 0.5
    threshold_pos = 1
    threshold_angle = 5/180 * 3.14

    rc_publisher = RobotControllerPublisher()
    rc_publisher.send_robot_pos(rc_publisher.robot_position)
    time.sleep(15)

    while True:
        future = rc_publisher.read_april_tag("Test")
        rclpy.spin_until_future_complete(rc_publisher, future)

        april_tag_position = future.result()
        rc_publisher.get_logger().info(f"Response: {april_tag_position.pose}, Robot_pos: {rc_publisher.robot_position}")

        error = np.array(april_tag_position.pose) - np.array(camera_dist)

        t_base_tool = crd_to_tm(rc_publisher.robot_position)
        t_tool_april = crd_to_tm(error)

        t_base_april = np.dot(t_base_tool, t_tool_april)

        base_april_crd = tm_to_crd(t_base_april)

        robot_pos = np.array(rc_publisher.robot_position) + (np.array(base_april_crd) - np.array(rc_publisher.robot_position)) * k

        robot_pos = robot_pos.tolist()

        rc_publisher.robot_position = robot_pos
        future = rc_publisher.send_robot_pos(robot_pos)
        rclpy.spin_until_future_complete(rc_publisher, future)


        time.sleep(0.5)

        while rc_publisher.robot_running:
            rclpy.spin_once(rc_publisher)
            continue

        rc_publisher.robot_state = True

        error_len = np.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
        error_angle = np.sqrt(error[3]**2 + error[4]**2 + error[5]**2)

        if error_len < threshold_pos and error_angle < threshold_angle:
            break

    rc_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
