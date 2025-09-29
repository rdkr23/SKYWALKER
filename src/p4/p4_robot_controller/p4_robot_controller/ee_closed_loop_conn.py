import rclpy
import time
import numpy as np
from numpy.linalg import inv
from rclpy.node import Node
from xarm_msgs.srv import MoveCartesian
from sensor_msgs.msg import JointState
from custom_msg.srv import GetAprilPos


class EEClosedLoopConn(Node):
    def __init__(self):
        super().__init__('robot_controller_publisher')

        self.robot_running = True

        self.robot_position = [462, 5, 600, -3.14 / 2, 0, -3.14 / 2]
        # self.robot_position = [462, 0, 711, 0, 0, 0]

        self.robot_state = self.create_subscription(JointState, '/xarm/joint_states', self.get_robot_state, 1)
        self.robot_state

        self.control_robot = self.create_client(MoveCartesian, '/xarm/set_position')
        self.april_tag_reader = self.create_client(GetAprilPos, '/get_tag_pose')

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
    x = tm[0, 3]
    y = tm[1, 3]
    z = tm[2, 3]

    rm = tm[0:3, 0:3]

    sy = np.sqrt(rm[0, 0] ** 2 + rm[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(rm[2, 1], rm[2, 2])
        pitch = np.arctan2(-rm[2, 0], sy)
        yaw = np.arctan2(rm[1, 0], rm[0, 0])
    else:
        roll = np.arctan2(-rm[1, 2], rm[1, 1])
        pitch = np.arctan2(-rm[2, 0], sy)
        yaw = 0

    return [x, y, z, roll, pitch, yaw]


def main(args=None):
    np.set_printoptions(suppress=True)
    rclpy.init(args=args)

    camera_dist = [0, 0, 200, 0, 0, 0]
    k = 0.3
    threshold_pos = 1
    threshold_angle = 5 / 180 * 3.14

    ee_clc = EEClosedLoopConn()
    ee_clc.send_robot_pos(ee_clc.robot_position)
    time.sleep(15)

    while True:
        future = ee_clc.read_april_tag("0")
        rclpy.spin_until_future_complete(ee_clc, future)

        april_tag_position = future.result()
        # april_tag_position.pose[3] = 0
        # april_tag_position.pose[4] = 0
        # april_tag_position.pose[5] = 0

        ee_clc.get_logger().info(f"Response: {np.round(np.array(april_tag_position.pose), 3)}")
        # ee_clc.get_logger().info(f"Robot_pos: {np.round(np.array(ee_clc.robot_position),3)}")

        error = np.array(april_tag_position.pose) - np.array(camera_dist)
        # ee_clc.get_logger().info(f"Error: {np.round(np.array(error), 3)}")
        t_base_camera = crd_to_tm(ee_clc.robot_position)

        # t_tool_camera = np.array([
        #     [np.cos(-np.pi/2), -np.sin(-np.pi/2), 0, 0],
        #     [np.sin(-np.pi/2), np.cos(-np.pi/2), 0, 0],
        #     [0, 0, 1, 0],
        #     [0, 0, 0, 1]
        # ])

        t_camera_april = crd_to_tm(error)

        t_base_april = np.dot(t_base_camera, t_camera_april)

        base_april_crd = tm_to_crd(t_base_april)

        robot_pos = np.array(ee_clc.robot_position) + (np.array(base_april_crd) - np.array(ee_clc.robot_position)) * k

        robot_pos = robot_pos.tolist()

        # ee_clc.get_logger().info(f"Final pos: {np.round(np.array(robot_pos),3)}")
        # robot_pos[0] = ee_clc.robot_position[0]
        # robot_pos[1] = ee_clc.robot_position[1]
        # robot_pos[2] = ee_clc.robot_position[2]
        # robot_pos[3] = ee_clc.robot_position[3]
        # robot_pos[4] = ee_clc.robot_position[4]
        # robot_pos[5] = ee_clc.robot_position[5]

        future = ee_clc.send_robot_pos(robot_pos)
        rclpy.spin_until_future_complete(ee_clc, future)
        ee_clc.robot_position = robot_pos

        time.sleep(0.5)

        while ee_clc.robot_running:
            rclpy.spin_once(ee_clc)
            continue

        # time.sleep(5)
        ee_clc.robot_state = True

        error_len = np.sqrt(error[0] ** 2 + error[1] ** 2 + error[2] ** 2)
        error_angle = np.sqrt(error[3] ** 2 + error[4] ** 2 + error[5] ** 2)

        # if error_len < threshold_pos and error_angle < threshold_angle:
        #     break

    ee_clc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
