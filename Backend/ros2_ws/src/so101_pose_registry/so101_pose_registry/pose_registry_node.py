from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

from builtin_interfaces.msg import Duration, Time as TimeMsg
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_follower_control.cartesian_kinematics import CartesianKinematics, frame_id_to_link_name
from so101_follower_control.runtime_utils import frame_to_pose_dict, gripper_position_to_fraction, workspace_bounds_to_dict
from so101_pose_registry.registry_store import StoredNamedPose, load_named_poses, resolve_current_named_pose_name, save_named_poses
from so101_pose_registry_interfaces.msg import NamedPose, WorkspaceBounds
from so101_pose_registry_interfaces.srv import DeleteNamedPose, GetNamedPose, ListNamedPoses, MoveToNamedPose, SaveCurrentPose


@dataclass(frozen=True)
class WorkspaceBoundsConfig:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def as_message(self) -> WorkspaceBounds:
        msg = WorkspaceBounds()
        msg.x_min = float(self.x_min)
        msg.x_max = float(self.x_max)
        msg.y_min = float(self.y_min)
        msg.y_max = float(self.y_max)
        msg.z_min = float(self.z_min)
        msg.z_max = float(self.z_max)
        return msg

    def as_dict(self) -> dict[str, float]:
        return workspace_bounds_to_dict(
            x_min=self.x_min,
            x_max=self.x_max,
            y_min=self.y_min,
            y_max=self.y_max,
            z_min=self.z_min,
            z_max=self.z_max,
        )


class PoseRegistryNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_registry_node')

        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('robot_description_topic', 'robot_description')
        self.declare_parameter('trajectory_topic', 'arm_controller/joint_trajectory')
        self.declare_parameter('gripper_command_topic', 'gripper_command')
        self.declare_parameter('catalog_topic', 'named_pose_catalog')
        self.declare_parameter('pose_file', str(Path('~/.ros/so101_follower/named_poses.yaml').expanduser()))
        self.declare_parameter('catalog_publish_rate', 1.0)
        self.declare_parameter('default_move_duration_sec', 3.0)
        self.declare_parameter('named_pose_match_tolerance_rad', 0.08)
        self.declare_parameter('gripper_joint_name', 'gripper')
        self.declare_parameter('gripper_closed_position', 0.0)
        self.declare_parameter('gripper_open_position', 1.0)
        self.declare_parameter('cartesian_base_frame', 'follower/base_link')
        self.declare_parameter('cartesian_tool_frame', 'follower/gripper_frame_link')
        self.declare_parameter('workspace_x_min', -0.3)
        self.declare_parameter('workspace_x_max', 0.5)
        self.declare_parameter('workspace_y_min', -0.4)
        self.declare_parameter('workspace_y_max', 0.4)
        self.declare_parameter('workspace_z_min', 0.0)
        self.declare_parameter('workspace_z_max', 0.5)

        self._joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self._robot_description_topic = str(self.get_parameter('robot_description_topic').value)
        self._trajectory_topic = str(self.get_parameter('trajectory_topic').value)
        self._gripper_command_topic = str(self.get_parameter('gripper_command_topic').value)
        self._catalog_topic = str(self.get_parameter('catalog_topic').value)
        self._pose_file = str(self.get_parameter('pose_file').value)
        self._catalog_publish_rate = float(self.get_parameter('catalog_publish_rate').value)
        self._default_move_duration_sec = float(self.get_parameter('default_move_duration_sec').value)
        self._named_pose_match_tolerance_rad = float(self.get_parameter('named_pose_match_tolerance_rad').value)
        self._gripper_joint_name = str(self.get_parameter('gripper_joint_name').value)
        self._gripper_closed_position = float(self.get_parameter('gripper_closed_position').value)
        self._gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self._cartesian_base_frame = str(self.get_parameter('cartesian_base_frame').value)
        self._cartesian_tool_frame = str(self.get_parameter('cartesian_tool_frame').value)
        self._workspace_bounds = WorkspaceBoundsConfig(
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
        self._named_poses = load_named_poses(self._pose_file)

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
        catalog_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(JointState, self._joint_state_topic, self._joint_state_callback, joint_state_qos)
        self.create_subscription(String, self._robot_description_topic, self._robot_description_callback, description_qos)
        self._trajectory_publisher = self.create_publisher(JointTrajectory, self._trajectory_topic, 10)
        self._gripper_command_publisher = self.create_publisher(Float64, self._gripper_command_topic, 10)
        self._catalog_publisher = self.create_publisher(String, self._catalog_topic, catalog_qos)
        self.create_service(ListNamedPoses, 'list_named_poses', self._handle_list_named_poses)
        self.create_service(GetNamedPose, 'get_named_pose', self._handle_get_named_pose)
        self.create_service(SaveCurrentPose, 'save_current_pose', self._handle_save_current_pose)
        self.create_service(DeleteNamedPose, 'delete_named_pose', self._handle_delete_named_pose)
        self.create_service(MoveToNamedPose, 'move_to_named_pose', self._handle_move_to_named_pose)
        self.create_timer(max(0.1, 1.0 / max(0.1, self._catalog_publish_rate)), self._publish_catalog)

        self.get_logger().info(
            f'Pose registry ready with {len(self._named_poses)} stored poses at {self._pose_file}.'
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        self._joint_order = list(msg.name)
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = float(position)

    def _robot_description_callback(self, msg: String) -> None:
        if msg.data == self._last_robot_description:
            return
        try:
            self._kinematics = CartesianKinematics.from_urdf_string(
                msg.data,
                base_link=frame_id_to_link_name(self._cartesian_base_frame),
                tool_link=frame_id_to_link_name(self._cartesian_tool_frame),
            )
            self._last_robot_description = msg.data
        except Exception as exc:
            self._kinematics = None
            self.get_logger().warn(f'Failed to initialize pose registry kinematics: {exc}')

    def _handle_list_named_poses(self, _request: ListNamedPoses.Request, response: ListNamedPoses.Response) -> ListNamedPoses.Response:
        response.workspace_bounds = self._workspace_bounds.as_message()
        response.poses = [self._named_pose_to_msg(self._named_poses[name]) for name in sorted(self._named_poses.keys())]
        return response

    def _handle_get_named_pose(self, request: GetNamedPose.Request, response: GetNamedPose.Response) -> GetNamedPose.Response:
        pose = self._named_poses.get(request.name)
        if pose is None:
            response.found = False
            response.message = f'Named pose {request.name!r} was not found.'
            return response
        response.found = True
        response.message = 'ok'
        response.pose = self._named_pose_to_msg(pose)
        return response

    def _handle_save_current_pose(self, request: SaveCurrentPose.Request, response: SaveCurrentPose.Response) -> SaveCurrentPose.Response:
        name = request.name.strip()
        if not name:
            response.success = False
            response.message = 'Pose name must not be empty.'
            return response
        if name in self._named_poses and not request.overwrite:
            response.success = False
            response.message = f'Named pose {name!r} already exists. Use overwrite=true to replace it.'
            return response

        try:
            pose = self._capture_current_pose(
                name=name,
                use_gripper_override=bool(request.use_gripper_open_fraction),
                gripper_open_fraction=float(request.gripper_open_fraction),
            )
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response

        self._named_poses[name] = pose
        save_named_poses(self._pose_file, self._named_poses)
        response.success = True
        response.message = f'Saved named pose {name!r}.'
        response.pose = self._named_pose_to_msg(pose)
        self._publish_catalog()
        return response

    def _handle_delete_named_pose(self, request: DeleteNamedPose.Request, response: DeleteNamedPose.Response) -> DeleteNamedPose.Response:
        if request.name not in self._named_poses:
            response.success = False
            response.message = f'Named pose {request.name!r} was not found.'
            return response
        del self._named_poses[request.name]
        save_named_poses(self._pose_file, self._named_poses)
        response.success = True
        response.message = f'Deleted named pose {request.name!r}.'
        self._publish_catalog()
        return response

    def _handle_move_to_named_pose(self, request: MoveToNamedPose.Request, response: MoveToNamedPose.Response) -> MoveToNamedPose.Response:
        pose = self._named_poses.get(request.name)
        if pose is None:
            response.success = False
            response.message = f'Named pose {request.name!r} was not found.'
            return response

        duration_sec = float(request.duration_sec) if request.duration_sec > 0.0 else self._default_move_duration_sec
        self._publish_joint_trajectory(pose.joint_names, pose.joint_positions, duration_sec)
        self._publish_gripper_fraction(pose.gripper_open_fraction)
        response.success = True
        response.message = f'Sent named pose {request.name!r}.'
        response.pose = self._named_pose_to_msg(pose)
        return response

    def _capture_current_pose(self, *, name: str, use_gripper_override: bool, gripper_open_fraction: float) -> StoredNamedPose:
        if not self._joint_order:
            raise ValueError('Cannot save a named pose before joint states are available.')
        tool_pose = self._compute_tool_pose_dict()
        if tool_pose is None:
            raise ValueError('Cannot save a named pose before the current tool pose is available.')

        if use_gripper_override:
            fraction = max(0.0, min(1.0, float(gripper_open_fraction)))
        else:
            current_gripper_position = self._joint_positions.get(self._gripper_joint_name)
            fraction = gripper_position_to_fraction(
                position=current_gripper_position if current_gripper_position is not None else self._gripper_closed_position,
                closed_position=self._gripper_closed_position,
                open_position=self._gripper_open_position,
            )
            if fraction is None:
                fraction = 0.0

        joint_names = list(self._joint_order)
        joint_positions = [
            float(self._joint_positions[name])
            for name in joint_names
            if name in self._joint_positions
        ]
        return StoredNamedPose(
            name=name,
            joint_names=joint_names,
            joint_positions=joint_positions,
            base_frame=self._cartesian_base_frame,
            tool_frame=self._cartesian_tool_frame,
            tool_pose=tool_pose,
            gripper_open_fraction=float(fraction),
            recorded_at=datetime.now(timezone.utc).isoformat(),
        )

    def _compute_tool_pose_dict(self) -> dict[str, Any] | None:
        if self._kinematics is None:
            return None
        try:
            joint_positions = [self._joint_positions[name] for name in self._kinematics.joint_names]
            frame = self._kinematics.forward_kinematics(joint_positions)
        except Exception:
            return None
        return frame_to_pose_dict(frame)

    def _current_named_pose_name(self) -> str | None:
        return resolve_current_named_pose_name(
            self._joint_positions,
            self._named_poses,
            tolerance_rad=self._named_pose_match_tolerance_rad,
        )

    def _publish_catalog(self) -> None:
        payload = {
            'workspace_bounds': self._workspace_bounds.as_dict(),
            'pose_names': sorted(self._named_poses.keys()),
            'current_named_pose': self._current_named_pose_name(),
            'poses': [
                self._named_poses[name].to_yaml_dict()
                for name in sorted(self._named_poses.keys())
            ],
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._catalog_publisher.publish(msg)

    def _named_pose_to_msg(self, pose: StoredNamedPose) -> NamedPose:
        msg = NamedPose()
        msg.name = pose.name
        msg.joint_names = list(pose.joint_names)
        msg.joint_positions = [float(value) for value in pose.joint_positions]
        msg.base_frame = pose.base_frame
        msg.tool_frame = pose.tool_frame
        msg.tool_pose = self._pose_msg_from_dict(pose.tool_pose)
        msg.gripper_open_fraction = float(pose.gripper_open_fraction)
        msg.recorded_at = self._time_msg_from_iso8601(pose.recorded_at)
        return msg

    def _publish_joint_trajectory(self, joint_names: list[str], joint_positions: list[float], duration_sec: float) -> None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in joint_positions]
        point.time_from_start = duration_from_seconds(duration_sec)
        trajectory.points = [point]
        self._trajectory_publisher.publish(trajectory)

    def _publish_gripper_fraction(self, fraction: float) -> None:
        msg = Float64()
        msg.data = float(fraction)
        self._gripper_command_publisher.publish(msg)

    @staticmethod
    def _pose_msg_from_dict(raw: dict[str, Any]) -> Pose:
        msg = Pose()
        position = raw.get('position', {})
        orientation = raw.get('orientation', {})
        msg.position.x = float(position.get('x', 0.0))
        msg.position.y = float(position.get('y', 0.0))
        msg.position.z = float(position.get('z', 0.0))
        msg.orientation.x = float(orientation.get('x', 0.0))
        msg.orientation.y = float(orientation.get('y', 0.0))
        msg.orientation.z = float(orientation.get('z', 0.0))
        msg.orientation.w = float(orientation.get('w', 1.0))
        return msg

    @staticmethod
    def _time_msg_from_iso8601(value: str) -> TimeMsg:
        msg = TimeMsg()
        try:
            timestamp = datetime.fromisoformat(value)
        except ValueError:
            return msg
        epoch_seconds = timestamp.timestamp()
        msg.sec = int(epoch_seconds)
        msg.nanosec = int((epoch_seconds - msg.sec) * 1e9)
        return msg


def duration_from_seconds(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    nanoseconds = int((seconds - whole_seconds) * 1e9)
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseRegistryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
