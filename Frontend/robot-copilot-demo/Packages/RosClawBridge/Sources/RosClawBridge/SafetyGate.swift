import Foundation

/// Pre-dispatch safety validator (mirrors rosclaw safety/validator.ts).
/// Validates a command against current robot state before any ROS2 operations are dispatched.
public struct SafetyGate: Sendable {

    /// Known valid preset names. In production, populated from capability discovery.
    public var knownPresets: Set<String>
    /// Known valid object IDs for pick-and-place.
    public var knownObjects: Set<String>
    /// Known valid destination IDs.
    public var knownDestinations: Set<String>
    /// Workspace limits sourced from the backend oracle.
    public var workspaceBounds: WorkspaceBounds

    public init(
        knownPresets: Set<String> = [],
        knownObjects: Set<String> = [],
        knownDestinations: Set<String> = [],
        workspaceBounds: WorkspaceBounds = .followerDefault
    ) {
        self.knownPresets = knownPresets
        self.knownObjects = knownObjects
        self.knownDestinations = knownDestinations
        self.workspaceBounds = workspaceBounds
    }

    /// Validates a command against the current robot state.
    /// Returns `.allowed` or `.blocked(reason:)`.
    public func validate(_ command: RobotCommand, robotState: RobotState) -> SafetyResult {
        // Emergency stop always passes
        if case .emergencyStop = command { return .allowed }

        // Robot must be connected and enabled for motion commands
        guard robotState.connectionState == .connected else {
            return .blocked(reason: "Robot is not connected")
        }
        guard robotState.enabled else {
            return .blocked(reason: "Robot is not enabled")
        }
        guard !robotState.estopLatched else {
            return .blocked(reason: "Emergency stop is latched")
        }
        guard robotState.fault == nil else {
            return .blocked(reason: "Robot is in fault state: \(robotState.fault!)")
        }
        guard robotState.mode != .faulted else {
            return .blocked(reason: "Robot mode is faulted")
        }

        // Command-specific validation
        switch command {
        case .moveToPreset(let name):
            guard knownPresets.contains(name.lowercased()) else {
                return .blocked(reason: "Unknown preset '\(name)'. Known presets: \(knownPresets.sorted().joined(separator: ", "))")
            }
        case .pickAndPlace(let objectID, let destinationID):
            guard knownObjects.contains(objectID.lowercased()) else {
                return .blocked(reason: "Unknown object '\(objectID)'")
            }
            guard knownDestinations.contains(destinationID.lowercased()) else {
                return .blocked(reason: "Unknown destination '\(destinationID)'")
            }
        case .moveToCartesian(let x, let y, let z):
            guard (workspaceBounds.xMin...workspaceBounds.xMax).contains(x) else {
                return .blocked(
                    reason: "Cartesian X=\(x) is outside workspace bounds [\(workspaceBounds.xMin), \(workspaceBounds.xMax)]")
            }
            guard (workspaceBounds.yMin...workspaceBounds.yMax).contains(y) else {
                return .blocked(
                    reason: "Cartesian Y=\(y) is outside workspace bounds [\(workspaceBounds.yMin), \(workspaceBounds.yMax)]")
            }
            guard (workspaceBounds.zMin...workspaceBounds.zMax).contains(z) else {
                return .blocked(
                    reason: "Cartesian Z=\(z) is outside workspace bounds [\(workspaceBounds.zMin), \(workspaceBounds.zMax)]")
            }
        default:
            break
        }

        return .allowed
    }
}
