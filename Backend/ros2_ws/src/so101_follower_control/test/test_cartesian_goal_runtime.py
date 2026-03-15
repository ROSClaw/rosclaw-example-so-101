from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import PoseStamped

from so101_follower_control.cartesian_goal_runtime import (
    CartesianGoalSafetyConfig,
    WorkspaceBoundsConfig,
    classify_candidate_resolution,
    clamp_pose_to_bounds,
    max_joint_delta,
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
