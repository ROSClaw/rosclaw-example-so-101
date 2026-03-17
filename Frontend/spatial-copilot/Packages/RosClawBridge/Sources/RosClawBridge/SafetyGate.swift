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
        case .moveToCartesian(let target):
            let requestedMeters = target.requestedMeters
            switch target.referenceFrame {
            case .baseAbsolute:
                guard (workspaceBounds.xMin...workspaceBounds.xMax).contains(requestedMeters.x) else {
                    return .blocked(
                        reason: "Cartesian X=\(requestedMeters.x) m is outside workspace bounds [\(workspaceBounds.xMin), \(workspaceBounds.xMax)]"
                    )
                }
                guard (workspaceBounds.yMin...workspaceBounds.yMax).contains(requestedMeters.y) else {
                    return .blocked(
                        reason: "Cartesian Y=\(requestedMeters.y) m is outside workspace bounds [\(workspaceBounds.yMin), \(workspaceBounds.yMax)]"
                    )
                }
                guard (workspaceBounds.zMin...workspaceBounds.zMax).contains(requestedMeters.z) else {
                    return .blocked(
                        reason: "Cartesian Z=\(requestedMeters.z) m is outside workspace bounds [\(workspaceBounds.zMin), \(workspaceBounds.zMax)]"
                    )
                }
            case .toolRelative:
                guard let toolPosition = robotState.toolPose?.position else {
                    return .blocked(reason: "Live gripper pose is unavailable for tool-relative Cartesian moves")
                }

                let targetX = toolPosition.x + requestedMeters.x
                let targetY = toolPosition.y + requestedMeters.y
                let targetZ = toolPosition.z + requestedMeters.z

                guard (workspaceBounds.xMin...workspaceBounds.xMax).contains(targetX) else {
                    return .blocked(
                        reason: "Tool-relative X move would place the gripper outside workspace bounds [\(workspaceBounds.xMin), \(workspaceBounds.xMax)]"
                    )
                }
                guard (workspaceBounds.yMin...workspaceBounds.yMax).contains(targetY) else {
                    return .blocked(
                        reason: "Tool-relative Y move would place the gripper outside workspace bounds [\(workspaceBounds.yMin), \(workspaceBounds.yMax)]"
                    )
                }
                guard (workspaceBounds.zMin...workspaceBounds.zMax).contains(targetZ) else {
                    return .blocked(
                        reason: "Tool-relative Z move would place the gripper outside workspace bounds [\(workspaceBounds.zMin), \(workspaceBounds.zMax)]"
                    )
                }
            }
        default:
            break
        }

        return .allowed
    }

    public func safeRange(
        for referenceFrame: RobotCartesianReferenceFrame,
        robotState: RobotState
    ) -> RobotCartesianSafeRange? {
        switch referenceFrame {
        case .baseAbsolute:
            return RobotCartesianSafeRange(
                referenceFrame: .baseAbsolute,
                x: RobotCartesianAxisRange(
                    minCm: workspaceBounds.xMin * 100.0,
                    maxCm: workspaceBounds.xMax * 100.0
                ),
                y: RobotCartesianAxisRange(
                    minCm: workspaceBounds.yMin * 100.0,
                    maxCm: workspaceBounds.yMax * 100.0
                ),
                z: RobotCartesianAxisRange(
                    minCm: workspaceBounds.zMin * 100.0,
                    maxCm: workspaceBounds.zMax * 100.0
                )
            )
        case .toolRelative:
            guard let toolPosition = robotState.toolPose?.position else {
                return nil
            }
            return RobotCartesianSafeRange(
                referenceFrame: .toolRelative,
                x: RobotCartesianAxisRange(
                    minCm: (workspaceBounds.xMin - toolPosition.x) * 100.0,
                    maxCm: (workspaceBounds.xMax - toolPosition.x) * 100.0
                ),
                y: RobotCartesianAxisRange(
                    minCm: (workspaceBounds.yMin - toolPosition.y) * 100.0,
                    maxCm: (workspaceBounds.yMax - toolPosition.y) * 100.0
                ),
                z: RobotCartesianAxisRange(
                    minCm: (workspaceBounds.zMin - toolPosition.z) * 100.0,
                    maxCm: (workspaceBounds.zMax - toolPosition.z) * 100.0
                )
            )
        }
    }
}
