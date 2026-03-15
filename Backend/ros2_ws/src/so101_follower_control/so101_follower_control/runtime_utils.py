from __future__ import annotations

import math
from typing import Iterable


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


def workspace_bounds_to_dict(
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> dict[str, float]:
    return {
        'x_min': float(x_min),
        'x_max': float(x_max),
        'y_min': float(y_min),
        'y_max': float(y_max),
        'z_min': float(z_min),
        'z_max': float(z_max),
    }


def build_agent_state_payload(
    *,
    joint_names: Iterable[str],
    joint_positions: dict[str, float],
    gripper_joint_name: str,
    gripper_open_fraction: float | None,
    tool_pose: dict[str, dict[str, float]] | None,
    tool_frame: str,
    workspace_bounds: dict[str, float] | None = None,
    named_pose_names: Iterable[str] | None = None,
    current_named_pose: str | None = None,
    last_resolved_cartesian_goal: dict[str, object] | None = None,
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
    if workspace_bounds is not None:
        payload['workspace_bounds'] = dict(workspace_bounds)
    if named_pose_names is not None:
        payload['named_pose_names'] = [str(name) for name in named_pose_names]
    if current_named_pose:
        payload['current_named_pose'] = str(current_named_pose)
    if last_resolved_cartesian_goal is not None:
        payload['last_resolved_cartesian_goal'] = dict(last_resolved_cartesian_goal)
    return payload


def pose_position_distance_m(
    first: dict[str, dict[str, float]] | None,
    second: dict[str, dict[str, float]] | None,
) -> float | None:
    if first is None or second is None:
        return None
    try:
        dx = float(first['position']['x']) - float(second['position']['x'])
        dy = float(first['position']['y']) - float(second['position']['y'])
        dz = float(first['position']['z']) - float(second['position']['z'])
    except KeyError:
        return None
    return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
