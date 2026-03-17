from __future__ import annotations

import math

import PyKDL as kdl

from so101_follower_control.agent_bridge_node import (
    build_agent_state_payload,
    gripper_fraction_to_position,
    gripper_position_to_fraction,
    frame_to_pose_dict,
)


def test_frame_to_pose_dict_serializes_kdl_frame():
    frame = kdl.Frame(
        kdl.Rotation.Quaternion(0.0, 0.0, 0.0, 1.0),
        kdl.Vector(0.1, -0.2, 0.3),
    )

    pose = frame_to_pose_dict(frame)

    assert pose == {
        'position': {'x': 0.1, 'y': -0.2, 'z': 0.3},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
    }


def test_build_agent_state_payload_includes_ordered_joint_state_and_pose():
    payload = build_agent_state_payload(
        joint_names=['shoulder_pan', 'gripper'],
        joint_positions={'shoulder_pan': 0.25, 'gripper': 0.75},
        gripper_joint_name='gripper',
        gripper_open_fraction=0.75,
        tool_pose={
            'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        },
        tool_frame='follower/gripper_frame_link',
        tool_pose_status={
            'stage': 'ready',
            'summary': 'Tool pose ready.',
        },
    )

    assert payload['joint_order'] == ['shoulder_pan', 'gripper']
    assert payload['joint_positions'] == {'shoulder_pan': 0.25, 'gripper': 0.75}
    assert math.isclose(payload['gripper_position'], 0.75)
    assert math.isclose(payload['gripper_open_fraction'], 0.75)
    assert payload['tool_pose'] == {
        'frame_id': 'follower/gripper_frame_link',
        'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
    }
    assert payload['tool_pose_status'] == {
        'stage': 'ready',
        'summary': 'Tool pose ready.',
    }


def test_build_agent_state_payload_includes_registry_and_resolution_metadata():
    payload = build_agent_state_payload(
        joint_names=['shoulder_pan'],
        joint_positions={'shoulder_pan': 0.25},
        gripper_joint_name='gripper',
        gripper_open_fraction=None,
        tool_pose=None,
        tool_frame='follower/gripper_frame_link',
        workspace_bounds={
            'x_min': -0.3,
            'x_max': 0.5,
            'y_min': -0.4,
            'y_max': 0.4,
            'z_min': 0.0,
            'z_max': 0.5,
        },
        named_pose_names=['home', 'setup'],
        current_named_pose='home',
        last_resolved_cartesian_goal={
            'accepted': True,
            'reason': 'resolved to the nearest safe reachable pose',
        },
    )

    assert payload['workspace_bounds']['x_min'] == -0.3
    assert payload['named_pose_names'] == ['home', 'setup']
    assert payload['current_named_pose'] == 'home'
    assert payload['last_resolved_cartesian_goal']['accepted'] is True


def test_gripper_fraction_to_position_maps_unit_interval_to_joint_space():
    assert math.isclose(
        gripper_fraction_to_position(
            open_fraction=0.0,
            closed_position=0.2,
            open_position=1.2,
        ),
        0.2,
    )
    assert math.isclose(
        gripper_fraction_to_position(
            open_fraction=1.0,
            closed_position=0.2,
            open_position=1.2,
        ),
        1.2,
    )
    assert math.isclose(
        gripper_fraction_to_position(
            open_fraction=0.25,
            closed_position=0.2,
            open_position=1.2,
        ),
        0.45,
    )


def test_gripper_position_to_fraction_clamps_and_handles_zero_span():
    assert math.isclose(
        gripper_position_to_fraction(
            position=0.7,
            closed_position=0.2,
            open_position=1.2,
        ),
        0.5,
    )
    assert math.isclose(
        gripper_position_to_fraction(
            position=2.0,
            closed_position=0.2,
            open_position=1.2,
        ),
        1.0,
    )
    assert (
        gripper_position_to_fraction(
            position=0.5,
            closed_position=1.0,
            open_position=1.0,
        )
        is None
    )
