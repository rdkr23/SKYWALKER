# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from custom_msg.srv import GetAprilPos


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher') # Name of node
        self.srv = self.create_service(GetAprilPos, 'april_tag', self.test)

        self.pose = [0, 0, 300, 0, 0, 0]

    def test(self, req, resp):
        resp.pose = self.pose
        self.get_logger().info(f"April tag id: {req.april_tag_id}")
        self.pose[2] *= 0.5
        if self.pose[2] < 50:
            self.pose[2] = 50
        return resp

        # self.publisher_ = self.create_publisher(PositionArray, 'camera_data', 10)
        #
        # msg = PositionArray()
        # msg.pose = [100,0,0,0,0,0]
        # self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
