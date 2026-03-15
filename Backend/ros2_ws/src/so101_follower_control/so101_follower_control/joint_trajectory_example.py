from builtin_interfaces.msg import Duration
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from so101_follower_control import JOINT_NAMES


class JointTrajectoryExample(Node):
    def __init__(self) -> None:
        super().__init__('joint_trajectory_example')
        self.declare_parameter(
            'action_name',
            '/follower/arm_controller/follow_joint_trajectory',
        )
        self.declare_parameter('positions', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('duration_sec', 3.0)

        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._positions = list(
            self.get_parameter('positions').get_parameter_value().double_array_value
        )
        self._duration_sec = (
            self.get_parameter('duration_sec').get_parameter_value().double_value
        )

        self._client = ActionClient(self, FollowJointTrajectory, action_name)

    def run(self) -> int:
        if len(self._positions) != len(JOINT_NAMES):
            self.get_logger().error(
                f'Expected {len(JOINT_NAMES)} joint positions, got {len(self._positions)}.'
            )
            return 1

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm trajectory action server is not available.')
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = self._positions
        seconds = int(self._duration_sec)
        nanoseconds = int((self._duration_sec - seconds) * 1e9)
        point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        goal.trajectory.points = [point]

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Arm trajectory goal was rejected.')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error('Failed to receive arm trajectory result.')
            return 1

        status = result.status
        error_code = result.result.error_code
        if error_code != 0:
            self.get_logger().error(
                f'Arm trajectory failed with status={status}, error_code={error_code}.'
            )
            return 1

        self.get_logger().info('Arm trajectory completed successfully.')
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointTrajectoryExample()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()
