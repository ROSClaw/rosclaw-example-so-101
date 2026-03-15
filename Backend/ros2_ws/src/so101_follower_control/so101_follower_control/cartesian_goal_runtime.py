from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PointStamped, PoseStamped

from so101_follower_control.cartesian_kinematics import (
    frame_id_to_link_name,
    normalize_pose_stamped,
    pose_to_kdl_frame,
)
from so101_follower_control.runtime_utils import frame_to_pose_dict

SO101_CARTESIAN_LINEAR_UNIT = 'm'
_CENTIMETERS_PER_METER = 100.0


@dataclass(frozen=True)
class WorkspaceBoundsConfig:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def clamp(self, x: float, y: float, z: float) -> tuple[tuple[float, float, float], float]:
        clamped_x = min(max(float(x), self.x_min), self.x_max)
        clamped_y = min(max(float(y), self.y_min), self.y_max)
        clamped_z = min(max(float(z), self.z_min), self.z_max)
        delta = math.sqrt(
            ((clamped_x - float(x)) ** 2)
            + ((clamped_y - float(y)) ** 2)
            + ((clamped_z - float(z)) ** 2)
        )
        return (clamped_x, clamped_y, clamped_z), delta

    def as_dict(self) -> dict[str, float]:
        return {
            'x_min': float(self.x_min),
            'x_max': float(self.x_max),
            'y_min': float(self.y_min),
            'y_max': float(self.y_max),
            'z_min': float(self.z_min),
            'z_max': float(self.z_max),
        }


@dataclass(frozen=True)
class CartesianGoalSafetyConfig:
    workspace_bounds: WorkspaceBoundsConfig
    max_step_distance_m: float
    max_joint_delta_rad: float
    max_best_effort_position_error_m: float
    orientation_tolerance_rad: float
    safe_orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class CartesianGoalResolution:
    accepted: bool
    reason: str
    requested_pose: PoseStamped
    resolved_pose: PoseStamped | None
    joint_positions: list[float]
    position_error_m: float
    orientation_error_rad: float
    current_step_distance_m: float | None
    max_joint_delta_rad: float | None
    workspace_clamp_distance_m: float

    def to_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {
            'accepted': self.accepted,
            'reason': self.reason,
            'requested_pose': pose_stamped_to_dict(self.requested_pose),
            'joint_positions': [float(value) for value in self.joint_positions],
            'position_error_m': _finite_or_none(self.position_error_m),
            'orientation_error_rad': _finite_or_none(self.orientation_error_rad),
            'workspace_clamp_distance_m': float(self.workspace_clamp_distance_m),
        }
        if self.resolved_pose is not None:
            payload['resolved_pose'] = pose_stamped_to_dict(self.resolved_pose)
        if self.current_step_distance_m is not None:
            payload['current_step_distance_m'] = _finite_or_none(self.current_step_distance_m)
        if self.max_joint_delta_rad is not None:
            payload['max_joint_delta_rad'] = _finite_or_none(self.max_joint_delta_rad)
        return payload


def with_fixed_orientation(
    goal: PoseStamped,
    quaternion_xyzw: tuple[float, float, float, float],
) -> PoseStamped:
    normalized = normalize_pose_stamped(goal)
    pose = PoseStamped()
    pose.header = normalized.header
    pose.pose.position = normalized.pose.position
    pose.pose.orientation.x = float(quaternion_xyzw[0])
    pose.pose.orientation.y = float(quaternion_xyzw[1])
    pose.pose.orientation.z = float(quaternion_xyzw[2])
    pose.pose.orientation.w = float(quaternion_xyzw[3])
    return normalize_pose_stamped(pose)


def uses_tool_relative_frame(goal_frame_id: str, tool_frame_id: str) -> bool:
    return frame_id_to_link_name(goal_frame_id) == frame_id_to_link_name(tool_frame_id)


def resolved_goal_pose(
    transformed_goal: PoseStamped,
    *,
    original_frame_id: str,
    tool_frame_id: str,
    safe_orientation_xyzw: tuple[float, float, float, float],
) -> PoseStamped:
    if uses_tool_relative_frame(original_frame_id, tool_frame_id):
        return normalize_pose_stamped(transformed_goal)
    return with_fixed_orientation(transformed_goal, safe_orientation_xyzw)


def normalize_goal_frame_id(frame_id: str, default_frame_id: str) -> str:
    normalized = frame_id.strip()
    return normalized if normalized else default_frame_id


def centimeters_to_cartesian_units(
    x_cm: float,
    y_cm: float,
    z_cm: float,
) -> tuple[float, float, float]:
    return (
        float(x_cm) / _CENTIMETERS_PER_METER,
        float(y_cm) / _CENTIMETERS_PER_METER,
        float(z_cm) / _CENTIMETERS_PER_METER,
    )


def point_stamped_from_centimeters(
    x_cm: float,
    y_cm: float,
    z_cm: float,
    *,
    frame_id: str,
    stamp: TimeMsg | None = None,
) -> PointStamped:
    stamped = PointStamped()
    stamped.header.frame_id = frame_id
    if stamp is not None:
        stamped.header.stamp = stamp
    stamped.point.x, stamped.point.y, stamped.point.z = centimeters_to_cartesian_units(
        x_cm,
        y_cm,
        z_cm,
    )
    return stamped


def clamp_pose_to_bounds(
    goal: PoseStamped,
    workspace_bounds: WorkspaceBoundsConfig,
) -> tuple[PoseStamped, float]:
    clamped_goal = PoseStamped()
    clamped_goal.header = goal.header
    clamped_goal.pose.orientation = goal.pose.orientation
    (x, y, z), clamp_distance = workspace_bounds.clamp(
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.position.z,
    )
    clamped_goal.pose.position.x = x
    clamped_goal.pose.position.y = y
    clamped_goal.pose.position.z = z
    return clamped_goal, clamp_distance


def pose_stamped_to_dict(pose: PoseStamped) -> dict[str, object]:
    return {
        'frame_id': pose.header.frame_id,
        **frame_to_pose_dict(pose_to_kdl_frame(pose.pose)),
    }


def pose_distance_m(first: PoseStamped, second: PoseStamped) -> float:
    dx = float(first.pose.position.x) - float(second.pose.position.x)
    dy = float(first.pose.position.y) - float(second.pose.position.y)
    dz = float(first.pose.position.z) - float(second.pose.position.z)
    return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))


def max_joint_delta(current_positions: Iterable[float], target_positions: Iterable[float]) -> float:
    deltas = [
        abs(float(current) - float(target))
        for current, target in zip(current_positions, target_positions)
    ]
    return max(deltas) if deltas else 0.0


def pose_stamped_from_pose_dict(
    pose: dict[str, object],
    *,
    frame_id: str,
    stamp: TimeMsg | None = None,
) -> PoseStamped:
    stamped = PoseStamped()
    stamped.header.frame_id = frame_id
    if stamp is not None:
        stamped.header.stamp = stamp
    position = pose.get('position', {})
    orientation = pose.get('orientation', {})
    stamped.pose.position.x = float(position.get('x', 0.0))
    stamped.pose.position.y = float(position.get('y', 0.0))
    stamped.pose.position.z = float(position.get('z', 0.0))
    stamped.pose.orientation.x = float(orientation.get('x', 0.0))
    stamped.pose.orientation.y = float(orientation.get('y', 0.0))
    stamped.pose.orientation.z = float(orientation.get('z', 0.0))
    stamped.pose.orientation.w = float(orientation.get('w', 1.0))
    return stamped


def _finite_or_none(value: float) -> float | None:
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def classify_candidate_resolution(
    *,
    position_error_m: float,
    orientation_error_rad: float,
    current_step_distance_m: float,
    joint_delta_rad: float,
    workspace_clamp_distance_m: float,
    safety: CartesianGoalSafetyConfig,
    exact_position_tolerance_m: float,
) -> tuple[bool, str]:
    if current_step_distance_m > safety.max_step_distance_m:
        return (
            False,
            'requested move exceeds the configured Cartesian step safety limit '
            f'({current_step_distance_m:.4f} m > {safety.max_step_distance_m:.4f} m)',
        )
    if joint_delta_rad > safety.max_joint_delta_rad:
        return (
            False,
            'requested move exceeds the configured per-joint delta safety limit '
            f'({joint_delta_rad:.4f} rad > {safety.max_joint_delta_rad:.4f} rad)',
        )
    if orientation_error_rad > safety.orientation_tolerance_rad:
        return (
            False,
            'best reachable pose exceeds the allowed orientation error '
            f'({orientation_error_rad:.4f} rad > {safety.orientation_tolerance_rad:.4f} rad)',
        )
    if position_error_m > safety.max_best_effort_position_error_m:
        return (
            False,
            'best reachable pose is too far from the requested point '
            f'({position_error_m:.4f} m > {safety.max_best_effort_position_error_m:.4f} m)',
        )
    if position_error_m <= exact_position_tolerance_m and workspace_clamp_distance_m <= 1e-6:
        return True, 'exact cartesian target resolved within tolerance'
    return True, 'resolved to the nearest safe reachable pose'
