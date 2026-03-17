import Foundation

// MARK: - Real ROS2 interface constants for the SO-101 follower arm

enum FollowerTopics {
    nonisolated static let jointStates = "/follower/joint_states"
    nonisolated static let cartesianGoal = "/follower/cartesian_goal"
    nonisolated static let armTrajectoryAction = "/follower/arm_controller/follow_joint_trajectory"
    nonisolated static let armTrajectoryActionType = "control_msgs/action/FollowJointTrajectory"
    nonisolated static let armTrajectoryTopic = "/follower/arm_controller/joint_trajectory"
    nonisolated static let gripperAction = "/follower/gripper_controller/gripper_cmd"
    nonisolated static let gripperActionType = "control_msgs/action/GripperCommand"
    nonisolated static let robotDescription = "/follower/robot_description"
    nonisolated static let baseFrame = "follower/base_link"
    nonisolated static let toolFrame = "follower/gripper_frame_link"
}

let followerJointNames = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

// MARK: - Preset joint configurations (radians — tune on real hardware)

enum FollowerPreset: String, CaseIterable, Sendable {
    case home
    case pickup
    case dropoff
    case rest

    var jointPositions: [Double] {
        switch self {
        case .home:    return [0.0,  0.0,  0.0,  0.0,  0.0]
        case .pickup:  return [0.0, -0.5, -0.8, -0.3,  0.0]
        case .dropoff: return [1.2, -0.3, -0.6, -0.2,  0.0]
        case .rest:    return [0.0,  0.3,  0.5,  0.3,  0.0]
        }
    }

    /// Gripper position after reaching this preset (1.0 = open, 0.0 = closed).
    var gripperPosition: Double {
        switch self {
        case .home:    return 1.0
        case .pickup:  return 1.0
        case .dropoff: return 1.0
        case .rest:    return 0.0
        }
    }
}

// MARK: - Rosbridge JSON message builders

/// Builds a `FollowJointTrajectory.Goal` payload for rosbridge `send_action_goal`.
func makeFollowJointTrajectoryGoal(
    positions: [Double],
    durationSec: Double = 3.0
) -> [String: Any] {
    let sec = Int(durationSec)
    let nanosec = Int((durationSec - Double(sec)) * 1e9)
    return [
        "trajectory": [
            "joint_names": followerJointNames,
            "points": [[
                "positions": positions,
                "time_from_start": ["sec": sec, "nanosec": nanosec],
            ] as [String: Any]],
        ] as [String: Any],
    ]
}

/// Builds a `GripperCommand.Goal` payload for rosbridge `send_action_goal`.
func makeGripperCommandGoal(
    position: Double,
    maxEffort: Double = 0.0
) -> [String: Any] {
    return [
        "command": [
            "position": position,
            "max_effort": maxEffort,
        ] as [String: Any],
    ]
}

/// Builds a `geometry_msgs/msg/PoseStamped` payload for rosbridge `publish`.
func makePoseStampedMsg(
    x: Double, y: Double, z: Double,
    qx: Double = 0.0, qy: Double = 0.707107, qz: Double = 0.0, qw: Double = 0.707107,
    frameId: String = FollowerTopics.baseFrame
) -> [String: Any] {
    return [
        "header": [
            "frame_id": frameId,
        ] as [String: Any],
        "pose": [
            "position": ["x": x, "y": y, "z": z],
            "orientation": ["x": qx, "y": qy, "z": qz, "w": qw],
        ] as [String: Any],
    ]
}
