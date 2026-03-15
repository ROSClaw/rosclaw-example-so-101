from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import PoseStamped

from so101_follower_control.cartesian_goal_runtime import (
    SO101_CARTESIAN_LINEAR_UNIT,
    CartesianGoalSafetyConfig,
    WorkspaceBoundsConfig,
    centimeters_to_cartesian_units,
    classify_candidate_resolution,
    clamp_pose_to_bounds,
    max_joint_delta,
    normalize_goal_frame_id,
    point_stamped_from_centimeters,
    resolved_goal_pose,
    uses_tool_relative_frame,
    with_fixed_orientation,
)


def test_clamp_pose_to_bounds_limits_out_of_workspace_targets():
    pose = PoseStamped()
    pose.header.frame_id = 'follower/base_link'
    pose.pose.position.x = 0.8
    pose.pose.position.y = -0.8
    pose.pose.position.z = -0.1
    pose.pose.orientation.w = 1.0

    clamped_pose, clamp_distance = clamp_pose_to_bounds(
        pose,
        WorkspaceBoundsConfig(
            x_min=-0.3,
            x_max=0.5,
            y_min=-0.4,
            y_max=0.4,
            z_min=0.0,
            z_max=0.5,
        ),
    )

    assert clamped_pose.pose.position.x == 0.5
    assert clamped_pose.pose.position.y == -0.4
    assert clamped_pose.pose.position.z == 0.0
    assert clamp_distance > 0.0


def test_with_fixed_orientation_overrides_requested_orientation():
    pose = PoseStamped()
    pose.header.frame_id = 'follower/base_link'
    pose.pose.position.x = 0.1
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0

    fixed = with_fixed_orientation(pose, (0.0, 0.707107, 0.0, 0.707107))

    assert fixed.pose.orientation.x == 0.0
    assert fixed.pose.orientation.y > 0.7
    assert fixed.pose.orientation.w > 0.7


def test_uses_tool_relative_frame_matches_tool_link_name():
    assert uses_tool_relative_frame('follower/gripper_frame_link', 'follower/gripper_frame_link')
    assert uses_tool_relative_frame('/gripper_frame_link', 'follower/gripper_frame_link')
    assert not uses_tool_relative_frame('follower/base_link', 'follower/gripper_frame_link')


def test_resolved_goal_pose_preserves_orientation_for_tool_relative_goals():
    pose = PoseStamped()
    pose.header.frame_id = 'follower/base_link'
    pose.pose.position.x = 0.1
    pose.pose.orientation.x = 0.1
    pose.pose.orientation.y = 0.2
    pose.pose.orientation.z = 0.3
    pose.pose.orientation.w = 0.9

    resolved = resolved_goal_pose(
        pose,
        original_frame_id='follower/gripper_frame_link',
        tool_frame_id='follower/gripper_frame_link',
        safe_orientation_xyzw=(0.0, 0.707107, 0.0, 0.707107),
    )

    assert resolved.pose.orientation.x != 0.0
    assert resolved.pose.orientation.y != 0.707107


def test_resolved_goal_pose_uses_fixed_orientation_for_base_frame_goals():
    pose = PoseStamped()
    pose.header.frame_id = 'follower/base_link'
    pose.pose.position.x = 0.1
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0

    resolved = resolved_goal_pose(
        pose,
        original_frame_id='follower/base_link',
        tool_frame_id='follower/gripper_frame_link',
        safe_orientation_xyzw=(0.0, 0.707107, 0.0, 0.707107),
    )

    assert resolved.pose.orientation.x == 0.0
    assert resolved.pose.orientation.y > 0.7
    assert resolved.pose.orientation.w > 0.7


def test_normalize_goal_frame_id_falls_back_to_default_frame():
    assert normalize_goal_frame_id('', 'follower/base_link') == 'follower/base_link'
    assert normalize_goal_frame_id('  ', 'follower/base_link') == 'follower/base_link'
    assert normalize_goal_frame_id('follower/gripper_frame_link', 'follower/base_link') == (
        'follower/gripper_frame_link'
    )


def test_centimeters_to_cartesian_units_returns_meters():
    assert centimeters_to_cartesian_units(15.0, -2.5, 0.0) == (0.15, -0.025, 0.0)


def test_point_stamped_from_centimeters_uses_so101_linear_unit():
    stamped = point_stamped_from_centimeters(
        12.5,
        0.0,
        -8.0,
        frame_id='follower/base_link',
    )

    assert SO101_CARTESIAN_LINEAR_UNIT == 'm'
    assert stamped.header.frame_id == 'follower/base_link'
    assert stamped.point.x == 0.125
    assert stamped.point.y == 0.0
    assert stamped.point.z == -0.08


def test_classify_candidate_resolution_accepts_exact_targets():
    safety = CartesianGoalSafetyConfig(
        workspace_bounds=WorkspaceBoundsConfig(-0.3, 0.5, -0.4, 0.4, 0.0, 0.5),
        max_step_distance_m=0.25,
        max_joint_delta_rad=1.1,
        max_best_effort_position_error_m=0.08,
        orientation_tolerance_rad=0.35,
        safe_orientation_xyzw=(0.0, 0.707107, 0.0, 0.707107),
    )

    accepted, reason = classify_candidate_resolution(
        position_error_m=0.01,
        orientation_error_rad=0.05,
        current_step_distance_m=0.10,
        joint_delta_rad=0.50,
        workspace_clamp_distance_m=0.0,
        safety=safety,
        exact_position_tolerance_m=0.02,
    )

    assert accepted is True
    assert reason == 'exact cartesian target resolved within tolerance'


def test_classify_candidate_resolution_accepts_best_effort_targets():
    safety = CartesianGoalSafetyConfig(
        workspace_bounds=WorkspaceBoundsConfig(-0.3, 0.5, -0.4, 0.4, 0.0, 0.5),
        max_step_distance_m=0.25,
        max_joint_delta_rad=1.1,
        max_best_effort_position_error_m=0.08,
        orientation_tolerance_rad=0.35,
        safe_orientation_xyzw=(0.0, 0.707107, 0.0, 0.707107),
    )

    accepted, reason = classify_candidate_resolution(
        position_error_m=0.05,
        orientation_error_rad=0.05,
        current_step_distance_m=0.10,
        joint_delta_rad=0.50,
        workspace_clamp_distance_m=0.03,
        safety=safety,
        exact_position_tolerance_m=0.02,
    )

    assert accepted is True
    assert reason == 'resolved to the nearest safe reachable pose'


def test_classify_candidate_resolution_rejects_unsafe_candidates():
    safety = CartesianGoalSafetyConfig(
        workspace_bounds=WorkspaceBoundsConfig(-0.3, 0.5, -0.4, 0.4, 0.0, 0.5),
        max_step_distance_m=0.25,
        max_joint_delta_rad=1.1,
        max_best_effort_position_error_m=0.08,
        orientation_tolerance_rad=0.35,
        safe_orientation_xyzw=(0.0, 0.707107, 0.0, 0.707107),
    )

    accepted, reason = classify_candidate_resolution(
        position_error_m=0.03,
        orientation_error_rad=0.05,
        current_step_distance_m=0.31,
        joint_delta_rad=0.50,
        workspace_clamp_distance_m=0.0,
        safety=safety,
        exact_position_tolerance_m=0.02,
    )

    assert accepted is False
    assert 'Cartesian step safety limit' in reason


def test_max_joint_delta_returns_largest_absolute_delta():
    assert max_joint_delta([0.0, 0.5, -0.25], [0.1, -0.2, 0.75]) == 1.0
