# import rclpy
# import time
# import numpy as np
# from numpy.linalg import inv
# from rclpy.node import Node
# from scipy.spatial.transform import Rotation
# from robot_controller.servo_robot_controller import ServoPlanner
# from tf2_ros import TransformListener, Buffer
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
#
#
# class ServoVisionController:
#     def __init__(self, node):
#         self.node = node
#
#         self.servo = ServoPlanner(self.node)
#         self.servo.initiate()
#
#         self.camera_offset_crd = [0, 0, 0, 0, 0, 0, 0] # The last four digits must be zero.
#
#         self.k = 0.2 # Regulator value
#
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self.node)
#
#         self.tf_broadcaster = TransformBroadcaster(self.node)
#
#         self.goal_pos = [0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0]
#
#     def create_frame(self):
#         t = TransformStamped()
#
#         t.header.stamp = self.node.get_clock().now().to_msg()
#         t.header.frame_id = 'link_base'
#         t.child_frame_id = 'servo_goal_frame'
#
#         t.transform.translation.x = self.goal_pos[0]
#         t.transform.translation.y = self.goal_pos[1]
#         t.transform.translation.z = self.goal_pos[2]
#         t.transform.rotation.x = self.goal_pos[3]
#         t.transform.rotation.y = self.goal_pos[4]
#         t.transform.rotation.z = self.goal_pos[5]
#         t.transform.rotation.w = self.goal_pos[6]
#
#         self.tf_broadcaster.sendTransform(t)
#
#     def run(self):
#         try:
#             rclpy.spin_once(self.node)
#             target_frame = "0"
#             transform = self.tf_buffer.lookup_transform('link7',f'tag36h11:{target_frame}', rclpy.time.Time()) #camera_color_optical_frame
#
#             at_translation = transform.transform.translation
#             at_rotation = transform.transform.rotation
#
#             ap = [at_translation.x, at_translation.y, at_translation.z, at_rotation.x, at_rotation.y, at_rotation.z,
#                     at_rotation.w]
#
#             # ap = [at_translation.x, at_translation.y, at_translation.z, 0, 0, 0, 1]
#
#             cp = self.servo.get_current_pose()
#             tm_base_camera = quad_to_tm(cp)
#
#
#             ap_error = np.array(ap) - np.array(self.camera_offset_crd)
#
#             tm_camera_tag = quad_to_tm(ap_error)
#
#             tm_tag_rot = quad_to_tm([0, 0, 0.2, 0.7071068, -0.7071068, 0, 0])
#
#             tm_base_tag = np.dot(tm_base_camera, np.dot(tm_camera_tag, tm_tag_rot))
#
#             base_april_pose = tm_to_quad(tm_base_tag)
#
#             robot_pos = np.array(cp) + (np.array(base_april_pose) - np.array(cp)) * self.k
#             robot_pos = robot_pos.tolist()
#
#             self.goal_pos = robot_pos
#
#             self.create_frame()
#
#             self.servo.set_final_pose(robot_pos)
#             self.servo.run()
#             # time.sleep(5)
#
#         except Exception as e:
#             self.node.get_logger().error(f'Could not get transforms: {e}')
#
#
# def quad_to_tm(pose):
#     r = Rotation.from_quat([pose[3], pose[4], pose[5], pose[6]])
#     r = r.as_matrix()
#
#     tm = np.vstack((np.hstack((r, [[pose[0]], [pose[1]], [pose[2]]])), [0, 0, 0, 1]))
#
#     return tm
#
#
# def tm_to_quad(tm):
#     x = tm[0, 3]
#     y = tm[1, 3]
#     z = tm[2, 3]
#
#     rot = tm[:3, :3]
#
#     r = Rotation.from_matrix(rot)
#     resp = [x, y, z] + r.as_quat().tolist()
#
#     return resp
#
#
# # def main(args=None):
# #     np.set_printoptions(suppress=True)
# #     rclpy.init(args=args)
# #
# #     camera_dist = [0, 0, 200, 0, 0, 0]
# #     k = 0.3
# #     threshold_pos = 1
# #     threshold_angle = 5 / 180 * 3.14
# #
# #     ee_clc = EEClosedLoopConn()
# #     ee_clc.send_robot_pos(ee_clc.robot_position)
# #     time.sleep(15)
# #
# #     while True:
# #         future = ee_clc.read_april_tag("0")
# #         rclpy.spin_until_future_complete(ee_clc, future)
# #
# #         april_tag_position = future.result()
# #         # april_tag_position.pose[3] = 0
# #         # april_tag_position.pose[4] = 0
# #         # april_tag_position.pose[5] = 0
# #
# #         ee_clc.get_logger().info(f"Response: {np.round(np.array(april_tag_position.pose), 3)}")
# #         # ee_clc.get_logger().info(f"Robot_pos: {np.round(np.array(ee_clc.robot_position),3)}")
# #
# #         error = np.array(april_tag_position.pose) - np.array(camera_dist)
# #         # ee_clc.get_logger().info(f"Error: {np.round(np.array(error), 3)}")
# #         t_base_camera = crd_to_tm(ee_clc.robot_position)
# #
# #         # t_tool_camera = np.array([
# #         #     [np.cos(-np.pi/2), -np.sin(-np.pi/2), 0, 0],
# #         #     [np.sin(-np.pi/2), np.cos(-np.pi/2), 0, 0],
# #         #     [0, 0, 1, 0],
# #         #     [0, 0, 0, 1]
# #         # ])
# #
# #         t_camera_april = crd_to_tm(error)
# #
# #         t_base_april = np.dot(t_base_camera, t_camera_april)
# #
# #         base_april_crd = tm_to_crd(t_base_april)
# #
# #         robot_pos = np.array(ee_clc.robot_position) + (np.array(base_april_crd) - np.array(ee_clc.robot_position)) * k
# #
# #         robot_pos = robot_pos.tolist()
# #
# #         # ee_clc.get_logger().info(f"Final pos: {np.round(np.array(robot_pos),3)}")
# #         # robot_pos[0] = ee_clc.robot_position[0]
# #         # robot_pos[1] = ee_clc.robot_position[1]
# #         # robot_pos[2] = ee_clc.robot_position[2]
# #         # robot_pos[3] = ee_clc.robot_position[3]
# #         # robot_pos[4] = ee_clc.robot_position[4]
# #         # robot_pos[5] = ee_clc.robot_position[5]
# #
# #         future = ee_clc.send_robot_pos(robot_pos)
# #         rclpy.spin_until_future_complete(ee_clc, future)
# #         ee_clc.robot_position = robot_pos
# #
# #         time.sleep(0.5)
# #
# #         while ee_clc.robot_running:
# #             rclpy.spin_once(ee_clc)
# #             continue
# #
# #         # time.sleep(5)
# #         ee_clc.robot_state = True
# #
# #         error_len = np.sqrt(error[0] ** 2 + error[1] ** 2 + error[2] ** 2)
# #         error_angle = np.sqrt(error[3] ** 2 + error[4] ** 2 + error[5] ** 2)
# #
# #         # if error_len < threshold_pos and error_angle < threshold_angle:
# #         #     break
# #
# #     ee_clc.destroy_node()
# #     rclpy.shutdown()
# #
# #
# # if __name__ == '__main__':
# #     main()
