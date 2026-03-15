from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import PoseStamped
import PyKDL as kdl

from so101_follower_control import JOINT_NAMES
from so101_follower_control.cartesian_kinematics import (
    CartesianKinematics,
    frame_id_to_link_name,
    normalize_pose_stamped,
)


def _description_path() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / 'so101_ros2'
        / 'so101_description'
        / 'urdf'
        / 'so101_new_calib.urdf'
    )


def _kinematics() -> CartesianKinematics:
    return CartesianKinematics.from_urdf_string(
        _description_path().read_bytes(),
        base_link='base_link',
        tool_link='gripper_frame_link',
    )


def test_frame_id_to_link_name_strips_prefixes():
    assert frame_id_to_link_name('follower/gripper_frame_link') == 'gripper_frame_link'
    assert frame_id_to_link_name('/follower/base_link') == 'base_link'
    assert frame_id_to_link_name('world') == 'world'


def test_normalize_pose_stamped_rejects_zero_quaternion():
    goal = PoseStamped()
    goal.header.frame_id = 'world'
    goal.pose.position.x = 0.1
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.0

    try:
        normalize_pose_stamped(goal)
    except ValueError as exc:
        assert 'non-zero quaternion' in str(exc)
    else:
        raise AssertionError('Expected zero quaternion validation to fail.')


def test_cartesian_kinematics_chain_matches_controller_joint_order():
    kinematics = _kinematics()
    assert kinematics.joint_names == JOINT_NAMES


def test_cartesian_ik_round_trips_a_reachable_pose():
    kinematics = _kinematics()
    target_positions = [0.1, -0.4, 0.5, -0.2, 0.3]
    target_frame = kinematics.forward_kinematics(target_positions)

    result = kinematics.solve_pose(target_frame, [0.0] * len(JOINT_NAMES))

    assert result is not None
    assert result.position_error_m < 1e-3
    assert result.orientation_error_rad < 1e-2


def test_cartesian_ik_keeps_large_unreachable_targets_outside_tolerance():
    kinematics = _kinematics()
    unreachable_frame = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(1.0, 0.0, 0.0))

    result = kinematics.solve_pose(unreachable_frame, [0.0] * len(JOINT_NAMES))

    assert result is not None
    assert result.position_error_m > 0.1
