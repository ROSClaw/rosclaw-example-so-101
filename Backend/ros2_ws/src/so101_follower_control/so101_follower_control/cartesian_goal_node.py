from __future__ import annotations

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
import json
import math
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_follower_control.cartesian_goal_runtime import (
    CartesianGoalResolution,
    CartesianGoalSafetyConfig,
    WorkspaceBoundsConfig,
    classify_candidate_resolution,
    clamp_pose_to_bounds,
    max_joint_delta,
    pose_distance_m,
    pose_stamped_from_pose_dict,
    with_fixed_orientation,
)
from so101_follower_control.cartesian_kinematics import (
    CartesianKinematics,
    frame_id_to_link_name,
    normalize_pose_stamped,
    pose_to_kdl_frame,
)
from so101_follower_control.runtime_utils import frame_to_pose_dict
from so101_pose_registry_interfaces.msg import ResolvedCartesianGoal as ResolvedCartesianGoalMsg
from so101_pose_registry_interfaces.srv import MoveToCartesianGoal, ResolveCartesianGoal


class CartesianGoalNode(Node):
    def __init__(self) -> None:
        super().__init__('cartesian_goal_node')

        self.declare_parameter('goal_topic', 'cartesian_goal')
        self.declare_parameter('trajectory_topic', 'arm_controller/joint_trajectory')
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('robot_description_topic', 'robot_description')
        self.declare_parameter('cartesian_base_frame', 'follower/base_link')
        self.declare_parameter('cartesian_tool_frame', 'follower/gripper_frame_link')
        self.declare_parameter('goal_duration_sec', 3.0)
        self.declare_parameter('position_tolerance_m', 0.02)
        self.declare_parameter('orientation_tolerance_rad', 0.35)
        self.declare_parameter('orientation_weight', 0.15)
        self.declare_parameter('ik_lambda', 0.05)
        self.declare_parameter('ik_max_iterations', 200)
        self.declare_parameter('max_cartesian_step_m', 0.25)
        self.declare_parameter('max_joint_delta_rad', 1.1)
        self.declare_parameter('max_best_effort_position_error_m', 0.08)
        self.declare_parameter('workspace_x_min', -0.3)
        self.declare_parameter('workspace_x_max', 0.5)
        self.declare_parameter('workspace_y_min', -0.4)
        self.declare_parameter('workspace_y_max', 0.4)
        self.declare_parameter('workspace_z_min', 0.0)
        self.declare_parameter('workspace_z_max', 0.5)
        self.declare_parameter('safe_orientation_x', 0.0)
        self.declare_parameter('safe_orientation_y', 0.707107)
        self.declare_parameter('safe_orientation_z', 0.0)
        self.declare_parameter('safe_orientation_w', 0.707107)
        self.declare_parameter('resolved_goal_topic', 'resolved_cartesian_goal')
        self.declare_parameter('resolve_cartesian_goal_service', 'resolve_cartesian_goal')
        self.declare_parameter('move_to_cartesian_goal_service', 'move_to_cartesian_goal')

        self._goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self._trajectory_topic = (
            self.get_parameter('trajectory_topic').get_parameter_value().string_value
        )
        self._joint_state_topic = (
            self.get_parameter('joint_state_topic').get_parameter_value().string_value
        )
        self._robot_description_topic = (
            self.get_parameter('robot_description_topic').get_parameter_value().string_value
        )
        self._cartesian_base_frame = (
            self.get_parameter('cartesian_base_frame').get_parameter_value().string_value
        )
        self._cartesian_tool_frame = (
            self.get_parameter('cartesian_tool_frame').get_parameter_value().string_value
        )
        self._goal_duration_sec = (
            self.get_parameter('goal_duration_sec').get_parameter_value().double_value
        )
        self._position_tolerance_m = (
            self.get_parameter('position_tolerance_m').get_parameter_value().double_value
        )
        self._orientation_tolerance_rad = (
            self.get_parameter('orientation_tolerance_rad').get_parameter_value().double_value
        )
        self._orientation_weight = (
            self.get_parameter('orientation_weight').get_parameter_value().double_value
        )
        self._ik_lambda = self.get_parameter('ik_lambda').get_parameter_value().double_value
        self._ik_max_iterations = (
            self.get_parameter('ik_max_iterations').get_parameter_value().integer_value
        )
        safe_orientation = (
            float(self.get_parameter('safe_orientation_x').value),
            float(self.get_parameter('safe_orientation_y').value),
            float(self.get_parameter('safe_orientation_z').value),
            float(self.get_parameter('safe_orientation_w').value),
        )
        self._safety = CartesianGoalSafetyConfig(
            workspace_bounds=WorkspaceBoundsConfig(
                x_min=float(self.get_parameter('workspace_x_min').value),
                x_max=float(self.get_parameter('workspace_x_max').value),
                y_min=float(self.get_parameter('workspace_y_min').value),
                y_max=float(self.get_parameter('workspace_y_max').value),
                z_min=float(self.get_parameter('workspace_z_min').value),
                z_max=float(self.get_parameter('workspace_z_max').value),
            ),
            max_step_distance_m=float(self.get_parameter('max_cartesian_step_m').value),
            max_joint_delta_rad=float(self.get_parameter('max_joint_delta_rad').value),
            max_best_effort_position_error_m=float(
                self.get_parameter('max_best_effort_position_error_m').value
            ),
            orientation_tolerance_rad=self._orientation_tolerance_rad,
            safe_orientation_xyzw=self._normalize_quaternion(safe_orientation),
        )
        self._resolved_goal_topic = str(self.get_parameter('resolved_goal_topic').value)
        self._resolve_service_name = str(self.get_parameter('resolve_cartesian_goal_service').value)
        self._move_service_name = str(self.get_parameter('move_to_cartesian_goal_service').value)

        self._tf_buffer: Buffer | None = None
        self._tf_listener: TransformListener | None = None
        self._kinematics: CartesianKinematics | None = None
        self._joint_positions: dict[str, float] = {}
        self._last_robot_description: str | None = None

        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        trajectory_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
        )
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
        resolved_goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self._trajectory_topic,
            trajectory_qos,
        )
        self._resolved_goal_publisher = self.create_publisher(
            String,
            self._resolved_goal_topic,
            resolved_goal_qos,
        )
        self.create_subscription(PoseStamped, self._goal_topic, self._goal_callback, goal_qos)
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
        self.create_service(
            ResolveCartesianGoal,
            self._resolve_service_name,
            self._handle_resolve_cartesian_goal,
        )
        self.create_service(
            MoveToCartesianGoal,
            self._move_service_name,
            self._handle_move_to_cartesian_goal,
        )

        self.get_logger().info(
            f'Listening for Cartesian goals for {self._cartesian_tool_frame} in '
            f'frame {self._cartesian_base_frame}.'
        )

    def _joint_state_callback(self, msg: JointState) -> None:
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
                orientation_weight=self._orientation_weight,
                damping=self._ik_lambda,
                max_iterations=self._ik_max_iterations,
            )
        except Exception as exc:
            self._kinematics = None
            self.get_logger().error(f'Failed to initialize Cartesian kinematics: {exc}')
            return

        self._last_robot_description = msg.data
        self.get_logger().info(
            f'Cartesian IK ready for {self._kinematics.base_link} -> '
            f'{self._kinematics.tool_link} using joints '
            f'[{", ".join(self._kinematics.joint_names)}].'
        )

    def _goal_callback(self, goal: PoseStamped) -> None:
        resolution = self._resolve_goal(goal)
        if resolution is None:
            return
        self._publish_resolution(resolution, executed=resolution.accepted)
        if not resolution.accepted:
            self.get_logger().warn(f'Rejecting Cartesian goal: {resolution.reason}')
            return
        self._publish_trajectory(resolution.joint_positions, self._goal_duration_sec)

    def _handle_resolve_cartesian_goal(
        self,
        request: ResolveCartesianGoal.Request,
        response: ResolveCartesianGoal.Response,
    ) -> ResolveCartesianGoal.Response:
        resolution = self._resolve_goal(request.pose)
        if resolution is None:
            resolution = self._internal_failure_resolution(
                request.pose,
                'Cartesian resolution failed before a candidate could be evaluated.',
            )
        self._publish_resolution(resolution, executed=False)
        response.result = self._resolution_to_msg(resolution)
        return response

    def _handle_move_to_cartesian_goal(
        self,
        request: MoveToCartesianGoal.Request,
        response: MoveToCartesianGoal.Response,
    ) -> MoveToCartesianGoal.Response:
        resolution = self._resolve_goal(request.pose)
        if resolution is None:
            resolution = self._internal_failure_resolution(
                request.pose,
                'Cartesian resolution failed before a candidate could be evaluated.',
            )
            self._publish_resolution(resolution, executed=False)
            response.result = self._resolution_to_msg(resolution)
            return response

        duration_sec = float(request.duration_sec) if request.duration_sec > 0.0 else self._goal_duration_sec
        if resolution.accepted:
            self._publish_trajectory(resolution.joint_positions, duration_sec)
        self._publish_resolution(resolution, executed=resolution.accepted)
        response.result = self._resolution_to_msg(resolution)
        return response

    def _resolve_goal(self, goal: PoseStamped) -> CartesianGoalResolution | None:
        if self._kinematics is None:
            self.get_logger().warn('Rejecting Cartesian goal because robot_description is not ready.')
            return self._internal_failure_resolution(goal, 'robot_description is not ready')

        if not goal.header.frame_id.strip():
            self.get_logger().warn('Rejecting Cartesian goal with an empty frame_id.')
            return self._internal_failure_resolution(goal, 'goal frame_id is empty')

        try:
            normalized_goal = normalize_pose_stamped(goal)
        except ValueError as exc:
            self.get_logger().warn(f'Rejecting Cartesian goal: {exc}')
            return self._internal_failure_resolution(goal, str(exc))

        transformed_goal = self._goal_in_base_frame(normalized_goal)
        if transformed_goal is None:
            return self._internal_failure_resolution(
                normalized_goal,
                f'could not transform goal into {self._cartesian_base_frame}',
            )

        try:
            seed_positions = [
                self._joint_positions[joint_name] for joint_name in self._kinematics.joint_names
            ]
        except KeyError:
            missing = [
                joint_name
                for joint_name in self._kinematics.joint_names
                if joint_name not in self._joint_positions
            ]
            reason = f'joint states are missing for {missing}'
            self.get_logger().warn(f'Rejecting Cartesian goal because {reason}.')
            return self._internal_failure_resolution(transformed_goal, reason)

        try:
            current_tool_frame = self._kinematics.forward_kinematics(seed_positions)
        except Exception as exc:
            reason = f'forward kinematics failed for the current tool pose: {exc}'
            self.get_logger().warn(f'Rejecting Cartesian goal because {reason}.')
            return self._internal_failure_resolution(transformed_goal, reason)

        requested_base_goal = with_fixed_orientation(
            transformed_goal,
            self._safety.safe_orientation_xyzw,
        )
        clamped_goal, workspace_clamp_distance_m = clamp_pose_to_bounds(
            requested_base_goal,
            self._safety.workspace_bounds,
        )

        result = self._kinematics.solve_pose(
            pose_to_kdl_frame(clamped_goal.pose),
            seed_positions,
        )
        if result is None:
            return CartesianGoalResolution(
                accepted=False,
                reason='inverse kinematics produced no candidate solution',
                requested_pose=requested_base_goal,
                resolved_pose=None,
                joint_positions=[],
                position_error_m=math.inf,
                orientation_error_rad=math.inf,
                current_step_distance_m=None,
                max_joint_delta_rad=None,
                workspace_clamp_distance_m=workspace_clamp_distance_m,
            )

        resolved_pose = pose_stamped_from_pose_dict(
            frame_to_pose_dict(self._kinematics.forward_kinematics(result.positions)),
            frame_id=self._cartesian_base_frame,
            stamp=requested_base_goal.header.stamp,
        )
        position_error_m = pose_distance_m(requested_base_goal, resolved_pose)
        step_distance_m = pose_distance_m(
            pose_stamped_from_pose_dict(
                frame_to_pose_dict(current_tool_frame),
                frame_id=self._cartesian_base_frame,
                stamp=requested_base_goal.header.stamp,
            ),
            resolved_pose,
        )
        candidate_joint_delta = max_joint_delta(seed_positions, result.positions)

        accepted, reason = classify_candidate_resolution(
            position_error_m=position_error_m,
            orientation_error_rad=result.orientation_error_rad,
            current_step_distance_m=step_distance_m,
            joint_delta_rad=candidate_joint_delta,
            workspace_clamp_distance_m=workspace_clamp_distance_m,
            safety=self._safety,
            exact_position_tolerance_m=self._position_tolerance_m,
        )

        return CartesianGoalResolution(
            accepted=accepted,
            reason=reason,
            requested_pose=requested_base_goal,
            resolved_pose=resolved_pose,
            joint_positions=result.positions,
            position_error_m=position_error_m,
            orientation_error_rad=result.orientation_error_rad,
            current_step_distance_m=step_distance_m,
            max_joint_delta_rad=candidate_joint_delta,
            workspace_clamp_distance_m=workspace_clamp_distance_m,
        )

    def _goal_in_base_frame(self, goal: PoseStamped) -> PoseStamped | None:
        if goal.header.frame_id == self._cartesian_base_frame:
            try:
                return normalize_pose_stamped(goal)
            except ValueError as exc:
                self.get_logger().warn(f'Rejecting Cartesian goal after normalization: {exc}')
                return None

        self._ensure_tf_listener()
        if self._tf_buffer is None:
            self.get_logger().error('TF buffer is not available for Cartesian goal transforms.')
            return None

        if goal.header.stamp.sec == 0 and goal.header.stamp.nanosec == 0:
            stamp = Time()
        else:
            stamp = Time.from_msg(goal.header.stamp)

        try:
            transform = self._tf_buffer.lookup_transform(
                self._cartesian_base_frame,
                goal.header.frame_id,
                stamp,
                timeout=RclpyDuration(seconds=0.2),
            )
            return normalize_pose_stamped(do_transform_pose_stamped(goal, transform))
        except TransformException as exc:
            self.get_logger().warn(
                'Rejecting Cartesian goal because TF could not transform '
                f'{goal.header.frame_id!r} into {self._cartesian_base_frame!r}: {exc}'
            )
            return None
        except ValueError as exc:
            self.get_logger().warn(f'Rejecting Cartesian goal after TF transform: {exc}')
            return None

    def _publish_trajectory(self, positions: list[float], duration_sec: float) -> None:
        if self._kinematics is None:
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self._kinematics.joint_names)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = duration_from_seconds(duration_sec)
        trajectory.points = [point]
        self._trajectory_publisher.publish(trajectory)

    def _publish_resolution(self, resolution: CartesianGoalResolution, *, executed: bool) -> None:
        payload = resolution.to_payload()
        payload['executed'] = executed
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._resolved_goal_publisher.publish(msg)

    def _resolution_to_msg(self, resolution: CartesianGoalResolution) -> ResolvedCartesianGoalMsg:
        msg = ResolvedCartesianGoalMsg()
        msg.accepted = bool(resolution.accepted)
        msg.reason = resolution.reason
        msg.requested_pose = resolution.requested_pose
        if resolution.resolved_pose is not None:
            msg.resolved_pose = resolution.resolved_pose
        msg.joint_positions = [float(value) for value in resolution.joint_positions]
        msg.position_error_m = float(resolution.position_error_m)
        msg.orientation_error_rad = float(resolution.orientation_error_rad)
        return msg

    def _internal_failure_resolution(self, goal: PoseStamped, reason: str) -> CartesianGoalResolution:
        try:
            normalized = normalize_pose_stamped(goal)
        except ValueError:
            normalized = PoseStamped()
            normalized.header = goal.header
            normalized.pose.position = goal.pose.position
            normalized.pose.orientation.w = 1.0
        return CartesianGoalResolution(
            accepted=False,
            reason=reason,
            requested_pose=normalized,
            resolved_pose=None,
            joint_positions=[],
            position_error_m=math.inf,
            orientation_error_rad=math.inf,
            current_step_distance_m=None,
            max_joint_delta_rad=None,
            workspace_clamp_distance_m=0.0,
        )

    def _ensure_tf_listener(self) -> None:
        if self._tf_buffer is not None:
            return

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

    @staticmethod
    def _normalize_quaternion(
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        norm = math.sqrt(sum(component * component for component in quaternion_xyzw))
        if norm <= 1e-9:
            raise ValueError('Configured safe_orientation quaternion must be non-zero.')
        return tuple(component / norm for component in quaternion_xyzw)


def duration_from_seconds(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    nanoseconds = int((seconds - whole_seconds) * 1e9)
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
