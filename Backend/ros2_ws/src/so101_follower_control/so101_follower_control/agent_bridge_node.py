from __future__ import annotations

import json
import math
from datetime import datetime, timezone
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
from so101_follower_control.runtime_utils import (
    build_agent_state_payload,
    clamp_unit_interval,
    frame_to_pose_dict,
    gripper_fraction_to_position,
    gripper_position_to_fraction,
    workspace_bounds_to_dict,
)


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
        self.declare_parameter('named_pose_catalog_topic', 'named_pose_catalog')
        self.declare_parameter('resolved_goal_topic', 'resolved_cartesian_goal')
        self.declare_parameter('workspace_x_min', -0.3)
        self.declare_parameter('workspace_x_max', 0.5)
        self.declare_parameter('workspace_y_min', -0.4)
        self.declare_parameter('workspace_y_max', 0.4)
        self.declare_parameter('workspace_z_min', 0.0)
        self.declare_parameter('workspace_z_max', 0.5)

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
        self._named_pose_catalog_topic = self.get_parameter('named_pose_catalog_topic').value
        self._resolved_goal_topic = self.get_parameter('resolved_goal_topic').value
        self._workspace_bounds = workspace_bounds_to_dict(
            x_min=float(self.get_parameter('workspace_x_min').value),
            x_max=float(self.get_parameter('workspace_x_max').value),
            y_min=float(self.get_parameter('workspace_y_min').value),
            y_max=float(self.get_parameter('workspace_y_max').value),
            z_min=float(self.get_parameter('workspace_z_min').value),
            z_max=float(self.get_parameter('workspace_z_max').value),
        )

        self._joint_order: list[str] = []
        self._joint_positions: dict[str, float] = {}
        self._last_robot_description: str | None = None
        self._kinematics: CartesianKinematics | None = None
        self._named_pose_names: list[str] = []
        self._current_named_pose: str | None = None
        self._last_resolved_cartesian_goal: dict[str, object] | None = None
        self._last_tool_pose_status_key: tuple[object, ...] | None = None
        self._has_logged_joint_states = False
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
        transient_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        self.create_subscription(
            String,
            self._named_pose_catalog_topic,
            self._named_pose_catalog_callback,
            transient_state_qos,
        )
        self.create_subscription(
            String,
            self._resolved_goal_topic,
            self._resolved_goal_callback,
            transient_state_qos,
        )
        self.create_service(Trigger, self._open_gripper_service, self._handle_open_gripper)
        self.create_service(Trigger, self._close_gripper_service, self._handle_close_gripper)
        self._state_publisher = self.create_publisher(String, self._state_topic, state_qos)
        self.create_timer(max(0.1, 1.0 / max(0.1, self._state_publish_rate)), self._publish_state)

        self.get_logger().info(
            f'Publishing SO-101 agent state on {self._state_topic}; '
            f'joint states from {self._joint_state_topic}; '
            f'robot description from {self._robot_description_topic}; '
            f'base/tool frames: {self._cartesian_base_frame} -> {self._cartesian_tool_frame}; '
            f'gripper commands on {self._gripper_command_topic}; '
            f'open/close services: {self._open_gripper_service}, {self._close_gripper_service}.'
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        self._joint_order = list(msg.name)
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position
        if not self._has_logged_joint_states:
            self._has_logged_joint_states = True
            self.get_logger().info(
                'Received initial joint state with joints [%s].'
                % ', '.join(self._joint_order)
            )

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
            self.get_logger().info(
                'Agent-state FK ready for %s -> %s using joints [%s].'
                % (
                    self._kinematics.base_link,
                    self._kinematics.tool_link,
                    ', '.join(self._kinematics.joint_names),
                )
            )
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

    def _named_pose_catalog_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring named pose catalog update because JSON is invalid: {exc}')
            return

        pose_names = payload.get('pose_names')
        if isinstance(pose_names, list):
            self._named_pose_names = [
                str(name).strip()
                for name in pose_names
                if str(name).strip()
            ]
        current_named_pose = payload.get('current_named_pose')
        self._current_named_pose = (
            str(current_named_pose).strip()
            if isinstance(current_named_pose, str) and str(current_named_pose).strip()
            else None
        )
        workspace_bounds = payload.get('workspace_bounds')
        if isinstance(workspace_bounds, dict):
            try:
                self._workspace_bounds = workspace_bounds_to_dict(
                    x_min=float(workspace_bounds.get('x_min', self._workspace_bounds['x_min'])),
                    x_max=float(workspace_bounds.get('x_max', self._workspace_bounds['x_max'])),
                    y_min=float(workspace_bounds.get('y_min', self._workspace_bounds['y_min'])),
                    y_max=float(workspace_bounds.get('y_max', self._workspace_bounds['y_max'])),
                    z_min=float(workspace_bounds.get('z_min', self._workspace_bounds['z_min'])),
                    z_max=float(workspace_bounds.get('z_max', self._workspace_bounds['z_max'])),
                )
            except (TypeError, ValueError):
                self.get_logger().warn('Ignoring named pose catalog workspace bounds because values are invalid.')

    def _resolved_goal_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring resolved goal update because JSON is invalid: {exc}')
            return
        if isinstance(payload, dict):
            self._last_resolved_cartesian_goal = payload

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
        tool_pose = self._compute_tool_pose()
        tool_pose_status = self._build_tool_pose_status(tool_pose)
        self._maybe_log_tool_pose_status(tool_pose_status)
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
            tool_pose_status=tool_pose_status,
            workspace_bounds=self._workspace_bounds,
            named_pose_names=self._named_pose_names,
            current_named_pose=self._current_named_pose,
            last_resolved_cartesian_goal=self._last_resolved_cartesian_goal,
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

    def _build_tool_pose_status(
        self,
        tool_pose: dict[str, dict[str, float]] | None,
    ) -> dict[str, object]:
        missing_joint_names: list[str] = []
        if self._kinematics is not None:
            missing_joint_names = [
                name for name in self._kinematics.joint_names if name not in self._joint_positions
            ]

        if not self._joint_order:
            stage = 'waiting_for_joint_states'
            summary = (
                f'Waiting for joint states on {self._joint_state_topic} before the tool pose can '
                'be computed.'
            )
        elif self._last_robot_description is None:
            stage = 'waiting_for_robot_description'
            summary = (
                f'Waiting for robot_description on {self._robot_description_topic} before '
                'forward kinematics can start.'
            )
        elif self._kinematics is None:
            stage = 'kinematics_unavailable'
            summary = (
                'Received robot_description but failed to initialize the forward-kinematics chain.'
            )
        elif missing_joint_names:
            stage = 'waiting_for_required_joints'
            summary = (
                'Waiting for required FK joints: %s.'
                % ', '.join(missing_joint_names)
            )
        elif tool_pose is None:
            stage = 'tool_pose_unavailable'
            summary = 'Joint states and kinematics are ready, but forward kinematics returned no tool pose.'
        else:
            stage = 'ready'
            position = tool_pose['position']
            summary = (
                'Tool pose ready in %s at x=%.3f m, y=%.3f m, z=%.3f m.'
                % (
                    self._cartesian_tool_frame,
                    float(position['x']),
                    float(position['y']),
                    float(position['z']),
                )
            )

        return {
            'stage': stage,
            'summary': summary,
            'joint_state_received': bool(self._joint_order),
            'joint_count': len(self._joint_positions),
            'robot_description_received': self._last_robot_description is not None,
            'kinematics_ready': self._kinematics is not None,
            'missing_joint_names': missing_joint_names,
            'base_frame': self._cartesian_base_frame,
            'tool_frame': self._cartesian_tool_frame,
            'updated_at': datetime.now(timezone.utc).isoformat(),
        }

    def _maybe_log_tool_pose_status(self, status: dict[str, object]) -> None:
        stage = str(status.get('stage', 'unknown'))
        summary = str(status.get('summary', ''))
        missing_joint_names = tuple(status.get('missing_joint_names', []))
        key = (stage, summary, missing_joint_names)
        if key == self._last_tool_pose_status_key:
            return

        self._last_tool_pose_status_key = key
        if stage in {'kinematics_unavailable', 'tool_pose_unavailable'}:
            self.get_logger().warn(f'Tool-pose pipeline: {summary}')
        else:
            self.get_logger().info(f'Tool-pose pipeline: {summary}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgentBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
