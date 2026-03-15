from __future__ import annotations

import json
import math
from typing import Iterable

from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from so101_follower_control.cartesian_kinematics import (
    CartesianKinematics,
    frame_id_to_link_name,
)


def frame_to_pose_dict(frame) -> dict[str, dict[str, float]]:
    quaternion = frame.M.GetQuaternion()
    return {
        'position': {
            'x': float(frame.p[0]),
            'y': float(frame.p[1]),
            'z': float(frame.p[2]),
        },
        'orientation': {
            'x': float(quaternion[0]),
            'y': float(quaternion[1]),
            'z': float(quaternion[2]),
            'w': float(quaternion[3]),
        },
    }


def build_agent_state_payload(
    *,
    joint_names: Iterable[str],
    joint_positions: dict[str, float],
    gripper_joint_name: str,
    gripper_open_fraction: float | None,
    tool_pose: dict[str, dict[str, float]] | None,
    tool_frame: str,
) -> dict[str, object]:
    ordered_joint_names = list(joint_names)
    payload: dict[str, object] = {
        'joint_order': ordered_joint_names,
        'joint_positions': {
            name: float(joint_positions[name])
            for name in ordered_joint_names
            if name in joint_positions
        },
    }
    if gripper_joint_name in joint_positions:
        payload['gripper_position'] = float(joint_positions[gripper_joint_name])
    if gripper_open_fraction is not None:
        payload['gripper_open_fraction'] = float(gripper_open_fraction)
    if tool_pose is not None:
        payload['tool_pose'] = {
            'frame_id': tool_frame,
            **tool_pose,
        }
    return payload


def clamp_unit_interval(value: float) -> float:
    return max(0.0, min(1.0, value))


def gripper_fraction_to_position(
    *,
    open_fraction: float,
    closed_position: float,
    open_position: float,
) -> float:
    fraction = clamp_unit_interval(open_fraction)
    return closed_position + ((open_position - closed_position) * fraction)


def gripper_position_to_fraction(
    *,
    position: float,
    closed_position: float,
    open_position: float,
) -> float | None:
    span = open_position - closed_position
    if abs(span) <= 1e-9:
        return None
    return clamp_unit_interval((position - closed_position) / span)


class AgentBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('agent_bridge_node')

        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('robot_description_topic', 'robot_description')
        self.declare_parameter('state_topic', 'agent_state')
        self.declare_parameter('state_publish_rate', 2.0)
        self.declare_parameter('gripper_command_topic', 'gripper_command')
        self.declare_parameter('gripper_action_name', 'gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_joint_name', 'gripper')
        self.declare_parameter('open_gripper_service', 'open_gripper')
        self.declare_parameter('close_gripper_service', 'close_gripper')
        self.declare_parameter('gripper_closed_position', 0.0)
        self.declare_parameter('gripper_open_position', 1.0)
        self.declare_parameter('gripper_max_effort', 10.0)
        self.declare_parameter('cartesian_base_frame', 'follower/base_link')
        self.declare_parameter('cartesian_tool_frame', 'follower/gripper_frame_link')

        self._joint_state_topic = self.get_parameter('joint_state_topic').value
        self._robot_description_topic = self.get_parameter('robot_description_topic').value
        self._state_topic = self.get_parameter('state_topic').value
        self._state_publish_rate = float(self.get_parameter('state_publish_rate').value)
        self._gripper_command_topic = self.get_parameter('gripper_command_topic').value
        self._gripper_action_name = self.get_parameter('gripper_action_name').value
        self._gripper_joint_name = self.get_parameter('gripper_joint_name').value
        self._open_gripper_service = self.get_parameter('open_gripper_service').value
        self._close_gripper_service = self.get_parameter('close_gripper_service').value
        self._gripper_closed_position = float(self.get_parameter('gripper_closed_position').value)
        self._gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self._gripper_max_effort = float(self.get_parameter('gripper_max_effort').value)
        self._cartesian_base_frame = self.get_parameter('cartesian_base_frame').value
        self._cartesian_tool_frame = self.get_parameter('cartesian_tool_frame').value

        self._joint_order: list[str] = []
        self._joint_positions: dict[str, float] = {}
        self._last_robot_description: str | None = None
        self._kinematics: CartesianKinematics | None = None
        self._gripper_action = ActionClient(self, GripperCommand, self._gripper_action_name)

        joint_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        description_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._joint_state_callback,
            joint_state_qos,
        )
        self.create_subscription(
            String,
            self._robot_description_topic,
            self._robot_description_callback,
            description_qos,
        )
        self.create_subscription(
            Float64,
            self._gripper_command_topic,
            self._gripper_command_callback,
            10,
        )
        self.create_service(Trigger, self._open_gripper_service, self._handle_open_gripper)
        self.create_service(Trigger, self._close_gripper_service, self._handle_close_gripper)
        self._state_publisher = self.create_publisher(String, self._state_topic, state_qos)
        self.create_timer(max(0.1, 1.0 / max(0.1, self._state_publish_rate)), self._publish_state)

        self.get_logger().info(
            f'Publishing SO-101 agent state on {self._state_topic} and listening for '
            f'gripper commands on {self._gripper_command_topic}. '
            f'Open/close services: {self._open_gripper_service}, {self._close_gripper_service}.'
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        self._joint_order = list(msg.name)
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position

    def _robot_description_callback(self, msg: String) -> None:
        if msg.data == self._last_robot_description:
            return

        try:
            base_link = frame_id_to_link_name(self._cartesian_base_frame)
            tool_link = frame_id_to_link_name(self._cartesian_tool_frame)
            self._kinematics = CartesianKinematics.from_urdf_string(
                msg.data,
                base_link=base_link,
                tool_link=tool_link,
            )
            self._last_robot_description = msg.data
        except Exception as exc:
            self._kinematics = None
            self.get_logger().warn(f'Failed to initialize agent-state kinematics: {exc}')

    def _gripper_command_callback(self, msg: Float64) -> None:
        open_fraction = float(msg.data)
        if not math.isfinite(open_fraction):
            self.get_logger().warn('Rejecting gripper command because the position is not finite.')
            return

        position = gripper_fraction_to_position(
            open_fraction=open_fraction,
            closed_position=self._gripper_closed_position,
            open_position=self._gripper_open_position,
        )
        self._send_gripper_goal(
            position=position,
            source=f'fraction command {clamp_unit_interval(open_fraction):.3f}',
        )

    def _handle_open_gripper(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        success = self._send_gripper_goal(
            position=self._gripper_open_position,
            source='open_gripper service',
        )
        response.success = success
        response.message = (
            'Sent gripper open goal.'
            if success
            else 'Gripper action server is unavailable.'
        )
        return response

    def _handle_close_gripper(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        success = self._send_gripper_goal(
            position=self._gripper_closed_position,
            source='close_gripper service',
        )
        response.success = success
        response.message = (
            'Sent gripper close goal.'
            if success
            else 'Gripper action server is unavailable.'
        )
        return response

    def _send_gripper_goal(self, *, position: float, source: str) -> bool:
        if not math.isfinite(position):
            self.get_logger().warn(f'Rejecting {source} because the target is not finite.')
            return False

        if not self._gripper_action.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'Rejecting {source} because the action server is unavailable.')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self._gripper_max_effort
        goal_future = self._gripper_action.send_goal_async(goal)
        goal_future.add_done_callback(
            lambda future: self._handle_gripper_goal_response(
                future,
                source=source,
                position=position,
            )
        )
        return True

    def _handle_gripper_goal_response(self, future, *, source: str, position: float) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Gripper goal from {source} failed to send: {exc}')
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                f'Gripper goal from {source} was rejected at position {position:.4f}.'
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._handle_gripper_result(
                result,
                source=source,
                position=position,
            )
        )

    def _handle_gripper_result(self, future, *, source: str, position: float) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Gripper goal from {source} failed while waiting for result: {exc}')
            return

        if result is None:
            self.get_logger().warn(f'Gripper goal from {source} returned no result.')
            return

        command_result = result.result
        self.get_logger().info(
            'Gripper goal from %s finished: target=%.4f reached_goal=%s stalled=%s position=%.4f'
            % (
                source,
                position,
                command_result.reached_goal,
                command_result.stalled,
                command_result.position,
            )
        )

    def _publish_state(self) -> None:
        if not self._joint_order:
            return

        tool_pose = self._compute_tool_pose()
        gripper_position = self._joint_positions.get(self._gripper_joint_name)
        gripper_open_fraction = None
        if gripper_position is not None:
            gripper_open_fraction = gripper_position_to_fraction(
                position=gripper_position,
                closed_position=self._gripper_closed_position,
                open_position=self._gripper_open_position,
            )
        payload = build_agent_state_payload(
            joint_names=self._joint_order,
            joint_positions=self._joint_positions,
            gripper_joint_name=self._gripper_joint_name,
            gripper_open_fraction=gripper_open_fraction,
            tool_pose=tool_pose,
            tool_frame=self._cartesian_tool_frame,
        )

        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._state_publisher.publish(msg)

    def _compute_tool_pose(self) -> dict[str, dict[str, float]] | None:
        if self._kinematics is None:
            return None

        try:
            joint_positions = [
                self._joint_positions[name]
                for name in self._kinematics.joint_names
            ]
        except KeyError:
            return None

        try:
            frame = self._kinematics.forward_kinematics(joint_positions)
        except Exception:
            return None
        return frame_to_pose_dict(frame)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgentBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
