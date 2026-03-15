from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class StoredNamedPose:
    name: str
    joint_names: list[str]
    joint_positions: list[float]
    base_frame: str
    tool_frame: str
    tool_pose: dict[str, Any]
    gripper_open_fraction: float
    recorded_at: str

    def to_yaml_dict(self) -> dict[str, Any]:
        return {
            'name': self.name,
            'joint_names': list(self.joint_names),
            'joint_positions': [float(value) for value in self.joint_positions],
            'base_frame': self.base_frame,
            'tool_frame': self.tool_frame,
            'tool_pose': self.tool_pose,
            'gripper_open_fraction': float(self.gripper_open_fraction),
            'recorded_at': self.recorded_at,
        }

    @classmethod
    def from_yaml_dict(cls, raw: dict[str, Any]) -> 'StoredNamedPose':
        return cls(
            name=str(raw['name']),
            joint_names=[str(value) for value in raw.get('joint_names', [])],
            joint_positions=[float(value) for value in raw.get('joint_positions', [])],
            base_frame=str(raw.get('base_frame', '')),
            tool_frame=str(raw.get('tool_frame', '')),
            tool_pose=dict(raw.get('tool_pose', {})),
            gripper_open_fraction=float(raw.get('gripper_open_fraction', 0.0)),
            recorded_at=str(raw.get('recorded_at', _utc_now_iso())),
        )


def load_named_poses(path: str | Path) -> dict[str, StoredNamedPose]:
    resolved = Path(path).expanduser()
    if not resolved.is_file():
        return {}

    raw = yaml.safe_load(resolved.read_text()) or {}
    pose_rows = raw.get('poses', [])
    poses: dict[str, StoredNamedPose] = {}
    for row in pose_rows:
        pose = StoredNamedPose.from_yaml_dict(row)
        poses[pose.name] = pose
    return poses


def save_named_poses(path: str | Path, poses: dict[str, StoredNamedPose]) -> None:
    resolved = Path(path).expanduser()
    resolved.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'poses': [
            poses[name].to_yaml_dict()
            for name in sorted(poses.keys())
        ]
    }
    resolved.write_text(yaml.safe_dump(payload, sort_keys=False))


def resolve_current_named_pose_name(
    current_joint_positions: dict[str, float],
    poses: dict[str, StoredNamedPose],
    *,
    tolerance_rad: float,
) -> str | None:
    if not poses:
        return None

    best_name: str | None = None
    best_max_delta: float | None = None
    for pose in poses.values():
        deltas: list[float] = []
        for joint_name, target_position in zip(pose.joint_names, pose.joint_positions):
            if joint_name not in current_joint_positions:
                deltas = []
                break
            deltas.append(abs(float(current_joint_positions[joint_name]) - float(target_position)))

        if not deltas:
            continue

        max_delta = max(deltas)
        if max_delta > tolerance_rad:
            continue
        if best_max_delta is None or max_delta < best_max_delta:
            best_name = pose.name
            best_max_delta = max_delta
    return best_name
