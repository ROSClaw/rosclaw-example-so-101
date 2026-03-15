from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_pose_registry.registry_store import StoredNamedPose, load_named_poses, resolve_current_named_pose_name, save_named_poses


def test_save_and_load_named_poses_round_trip(tmp_path):
    path = tmp_path / 'poses.yaml'
    poses = {
        'home': StoredNamedPose(
            name='home',
            joint_names=['a', 'b'],
            joint_positions=[0.1, 0.2],
            base_frame='base',
            tool_frame='tool',
            tool_pose={'position': {'x': 0.1, 'y': 0.2, 'z': 0.3}},
            gripper_open_fraction=0.75,
            recorded_at='2026-01-01T00:00:00+00:00',
        )
    }

    save_named_poses(path, poses)
    loaded = load_named_poses(path)

    assert loaded == poses


def test_resolve_current_named_pose_name_uses_tolerance_and_best_match():
    poses = {
        'home': StoredNamedPose(
            name='home',
            joint_names=['joint_1', 'joint_2'],
            joint_positions=[0.0, 0.0],
            base_frame='base',
            tool_frame='tool',
            tool_pose={},
            gripper_open_fraction=1.0,
            recorded_at='2026-01-01T00:00:00+00:00',
        ),
        'setup': StoredNamedPose(
            name='setup',
            joint_names=['joint_1', 'joint_2'],
            joint_positions=[0.5, 0.5],
            base_frame='base',
            tool_frame='tool',
            tool_pose={},
            gripper_open_fraction=0.0,
            recorded_at='2026-01-01T00:00:00+00:00',
        ),
    }

    assert resolve_current_named_pose_name({'joint_1': 0.02, 'joint_2': -0.01}, poses, tolerance_rad=0.05) == 'home'
    assert resolve_current_named_pose_name({'joint_1': 0.47, 'joint_2': 0.49}, poses, tolerance_rad=0.05) == 'setup'
    assert resolve_current_named_pose_name({'joint_1': 1.0, 'joint_2': 1.0}, poses, tolerance_rad=0.05) is None
