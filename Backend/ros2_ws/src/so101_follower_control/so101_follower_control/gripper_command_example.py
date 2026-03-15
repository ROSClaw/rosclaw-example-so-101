import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand


class GripperCommandExample(Node):
    def __init__(self) -> None:
        super().__init__('gripper_command_example')
        self.declare_parameter(
            'action_name',
            '/follower/gripper_controller/gripper_cmd',
        )
        self.declare_parameter('position', 1.0)
        self.declare_parameter('max_effort', 0.0)

        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._position = self.get_parameter('position').get_parameter_value().double_value
        self._max_effort = self.get_parameter('max_effort').get_parameter_value().double_value
        self._client = ActionClient(self, GripperCommand, action_name)

    def run(self) -> int:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server is not available.')
            return 1

        goal = GripperCommand.Goal()
        goal.command.position = self._position
        goal.command.max_effort = self._max_effort

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Gripper goal was rejected.')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error('Failed to receive gripper result.')
            return 1

        if not result.result.reached_goal:
            self.get_logger().error('Gripper command finished without reaching the goal.')
            return 1

        stalled = result.result.stalled
        self.get_logger().info(f'Gripper command completed. stalled={stalled}')
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperCommandExample()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()
