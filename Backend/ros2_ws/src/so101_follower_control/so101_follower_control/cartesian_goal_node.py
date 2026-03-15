from __future__ import annotations

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
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

from so101_follower_control.cartesian_kinematics import (
    CartesianKinematics,
    frame_id_to_link_name,
    normalize_pose_stamped,
    pose_to_kdl_frame,
)


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

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self._trajectory_topic,
            trajectory_qos,
        )
        self._goal_subscriber = self.create_subscription(
            PoseStamped,
            self._goal_topic,
            self._goal_callback,
            goal_qos,
        )
        self._joint_state_subscriber = self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._joint_state_callback,
            joint_state_qos,
        )
        self._robot_description_subscriber = self.create_subscription(
            String,
            self._robot_description_topic,
            self._robot_description_callback,
            description_qos,
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
        if self._kinematics is None:
            self.get_logger().warn('Rejecting Cartesian goal because robot_description is not ready.')
            return

        if not goal.header.frame_id.strip():
            self.get_logger().warn('Rejecting Cartesian goal with an empty frame_id.')
            return

        try:
            normalized_goal = normalize_pose_stamped(goal)
        except ValueError as exc:
            self.get_logger().warn(f'Rejecting Cartesian goal: {exc}')
            return

        transformed_goal = self._goal_in_base_frame(normalized_goal)
        if transformed_goal is None:
            return

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
            self.get_logger().warn(
                f'Rejecting Cartesian goal because joint states are missing for {missing}.'
            )
            return

        result = self._kinematics.solve_pose(
            pose_to_kdl_frame(transformed_goal.pose),
            seed_positions,
        )
        if result is None:
            self.get_logger().error('Cartesian IK failed without producing a candidate solution.')
            return

        if result.position_error_m > self._position_tolerance_m:
            self.get_logger().warn(
                'Rejecting Cartesian goal because the best IK solution is too far from the target '
                f'(position error={result.position_error_m:.4f} m, '
                f'orientation error={result.orientation_error_rad:.4f} rad).'
            )
            return

        if result.orientation_error_rad > self._orientation_tolerance_rad:
            self.get_logger().warn(
                'Rejecting Cartesian goal because the best IK solution misses the target orientation '
                f'(orientation error={result.orientation_error_rad:.4f} rad, '
                f'position error={result.position_error_m:.4f} m).'
            )
            return

        self._publish_trajectory(result.positions)

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

    def _publish_trajectory(self, positions: list[float]) -> None:
        if self._kinematics is None:
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self._kinematics.joint_names)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = duration_from_seconds(self._goal_duration_sec)
        trajectory.points = [point]
        self._trajectory_publisher.publish(trajectory)

    def _ensure_tf_listener(self) -> None:
        if self._tf_buffer is not None:
            return

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)


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
