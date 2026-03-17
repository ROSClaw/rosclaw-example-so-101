import Foundation

/// Constrained robot command model (§11.1).
/// The agent must normalize all user utterances into one of these cases
/// before any ROS2 operations are generated.
public enum RobotCommand: Sendable, Equatable {
    case homeRobot
    case openGripper
    case closeGripper
    case moveToPreset(name: String)
    case pickAndPlace(objectID: String, destinationID: String)
    case moveToCartesian(target: RobotCartesianTarget)
    case captureSnapshot(cameraID: String)
    case emergencyStop

    /// Human-readable display name for UI.
    public var displayName: String {
        switch self {
        case .homeRobot: return "Home Robot"
        case .openGripper: return "Open Gripper"
        case .closeGripper: return "Close Gripper"
        case .moveToPreset(let name): return "Move to \(name)"
        case .pickAndPlace(let obj, let dest): return "Pick \(obj) → \(dest)"
        case .moveToCartesian(let target):
            let verb = target.referenceFrame == .toolRelative ? "Move gripper by" : "Move to"
            return "\(verb) \(target.requestSummary) [\(target.frameDisplayName)]"
        case .captureSnapshot(let cam): return "Capture Snapshot (\(cam))"
        case .emergencyStop: return "Emergency Stop"
        }
    }

    /// Whether this command requires explicit user confirmation before execution.
    public var requiresConfirmation: Bool {
        switch self {
        case .emergencyStop: return false
        case .homeRobot, .openGripper, .closeGripper,
             .moveToPreset, .pickAndPlace, .captureSnapshot,
             .moveToCartesian:
            return true
        }
    }

    /// Subsystems this command touches (for preview display).
    public var subsystemsTouched: [String] {
        switch self {
        case .homeRobot: return ["arm", "controller"]
        case .openGripper, .closeGripper: return ["gripper"]
        case .moveToPreset: return ["arm", "controller"]
        case .pickAndPlace: return ["arm", "gripper", "controller"]
        case .moveToCartesian: return ["arm", "controller", "ik"]
        case .captureSnapshot: return ["camera"]
        case .emergencyStop: return ["arm", "gripper", "controller"]
        }
    }
}
