#!/home/rdk/ros2_pytorch_env/bin/python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


def dur(seconds: float) -> Duration:
    """Create a builtin_interfaces.msg.Duration from seconds (float)."""
    sec = int(seconds)
    nsec = int(round((seconds - sec) * 1e9))
    if nsec < 0:
        nsec = 0
    d = Duration()
    d.sec = sec
    d.nanosec = nsec
    return d


class JointGoalForwarder(Node):
   

    def __init__(self):
        super().__init__('joint_goal_forwarder')

        # ---- Parameters (with defaults) ----
        # Topic to subscribe for incoming JointTrajectory 
        self.declare_parameter('input_topic', '/joint_trajectory_controller/joint_trajectory')

        # controller's action name to forward the trajectory
        self.declare_parameter('output_topic', '/xarm7_traj_controller/follow_joint_trajectory')

        # Controller's expected joint order (match your controller names exactly)
        default_joint_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        self.declare_parameter('joint_order', default_joint_order)

        # Default move time if incoming first point has non-positive/absent time_from_start
        self.declare_parameter('move_time', 1.0)

        # Get params
        self.input_topic = self.get_parameter('input_topic').value
        self.action_name = self.get_parameter('output_topic').value  
        self.joint_order = list(self.get_parameter('joint_order').value)
        self.move_time = float(self.get_parameter('move_time').value)


        # ---- Action client ----
        self.ac = ActionClient(self, FollowJointTrajectory, self.action_name)
        self.get_logger().info(f'Waiting for action server on "{self.action_name}"...')
        self.ac.wait_for_server()
        self.get_logger().info('Action server ready.')

        # ---- Subscription ----
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(JointTrajectory, self.input_topic, self.on_input_trajectory, qos)

        self.active_goal = None  # kept only for logging (no cancel logic)

    # ---- Helpers ----
    def send_one_point(self, positions, duration_sec: float):
        """Build and send a single-point FollowJointTrajectory goal."""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_order

        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]

        if len(pt.positions) != len(traj.joint_names):
            self.get_logger().warn('Positions length does not match joint_names length; rejecting.')
            return

        # Ensure strictly positive duration
        use_time = duration_sec if (duration_sec is not None and duration_sec > 0.0) else self.move_time
        pt.time_from_start = dur(use_time)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        # Optional tolerance: uncomment if you want it
        # goal.goal_time_tolerance = dur(0.5)

        future = self.ac.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    # ---- Callbacks ----
    def on_input_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn('Received JointTrajectory with no points; ignoring.')
            return

        incoming_joint_names = list(msg.joint_names)
        first_point = msg.points[0]
        incoming_positions = list(first_point.positions)

        # Extract incoming duration (may be zero/negative/absent)
        incoming_duration = None
        try:
            incoming_duration = float(first_point.time_from_start.sec) + float(first_point.time_from_start.nanosec) / 1e9
            if incoming_duration <= 0.0:
                incoming_duration = None
        except Exception:
            incoming_duration = None

        # If exact order matches, just use it
        if incoming_joint_names == self.joint_order:
            out_positions = incoming_positions
        else:
            # Reorder strictly (no zero fill). Reject if missing any required joint.
            name_to_idx = {n: i for i, n in enumerate(incoming_joint_names)}
            out_positions = []
            for name in self.joint_order:
                if name not in name_to_idx:
                    self.get_logger().warn(f'Missing required joint "{name}" in incoming trajectory; rejecting message.')
                    return
                out_positions.append(float(incoming_positions[name_to_idx[name]]))

        self.send_one_point(out_positions, duration_sec=incoming_duration)

    def on_goal_response(self, future):
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().warn('FollowJointTrajectory goal was rejected.')
            return
        self.get_logger().info('FollowJointTrajectory goal accepted.')
        self.active_goal = gh
        gh.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        # rclpy passes a FeedbackMessage wrapper with .feedback
        try:
            fb = feedback_msg.feedback
            if fb.error.positions:
                self.get_logger().debug(f"pos err j1: {fb.error.positions[0]:.5f}")
        except Exception:
            pass

    def on_result(self, future):
        try:
            result = future.result().result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().warn(f"Goal finished with error {result.error_code}: {result.error_string}")
            else:
                self.get_logger().info('Goal succeeded.')
        except Exception as e:
            self.get_logger().warn(f'Failed to get result: {e}')

def main():
    rclpy.init()
    node = JointGoalForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
