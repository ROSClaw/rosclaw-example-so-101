import Foundation
import RosClawBridge

/// Live robot state wrapper — owns the mutable RobotState, safety gate,
/// and derived kinematics/planning data.
@MainActor
@Observable
final class RobotStateModel {

    var robotState = RobotState()
    var safetyGate = SafetyGate()
    var currentNamedPose: String?
    var lastResolvedCartesianGoal: GatewayResolvedCartesianGoal?
    var lastRequestedCartesianTarget: RobotCartesianTarget?
    var toolPoseStatus: GatewayToolPoseStatus?

    var toolPoseStatusSummary: String {
        toolPoseStatus?.summary ?? "No tool-pose status received yet."
    }

    // MARK: - Safe range helpers

    var baseAbsoluteSafeRangeCm: RobotCartesianSafeRange? {
        safetyGate.safeRange(for: .baseAbsolute, robotState: robotState)
    }

    var toolRelativeSafeRangeCm: RobotCartesianSafeRange? {
        safetyGate.safeRange(for: .toolRelative, robotState: robotState)
    }

    func safeRangeSummary(for referenceFrame: RobotCartesianReferenceFrame) -> String? {
        let range: RobotCartesianSafeRange?
        switch referenceFrame {
        case .baseAbsolute:
            range = baseAbsoluteSafeRangeCm
        case .toolRelative:
            range = toolRelativeSafeRangeCm
        }
        guard let range else { return nil }
        return String(
            format: "x [%.1f, %.1f] cm, y [%.1f, %.1f] cm, z [%.1f, %.1f] cm",
            range.x.minCm, range.x.maxCm,
            range.y.minCm, range.y.maxCm,
            range.z.minCm, range.z.maxCm
        )
    }

    func requestedCartesianSummary(_ target: RobotCartesianTarget?) -> String {
        guard let target else { return "Unavailable" }
        return "\(target.requestSummary) [\(target.frameDisplayName)]"
    }

    // MARK: - Follower state application

    func applyFollowerState(_ state: GatewayFollowerState, estopLatched: Bool) {
        robotState.jointOrder = state.jointOrder
        robotState.jointPositions = state.jointPositions
        robotState.gripperPosition = state.gripperPosition
        robotState.gripperOpenFraction = state.gripperOpenFraction
        robotState.toolPose = state.toolPose
        toolPoseStatus = state.toolPoseStatus
        currentNamedPose = state.currentNamedPose
        lastResolvedCartesianGoal = state.lastResolvedCartesianGoal
        if let stateWorkspaceBounds = state.workspaceBounds {
            safetyGate.workspaceBounds = stateWorkspaceBounds
        }
        if !state.namedPoseNames.isEmpty {
            let normalizedPresets = Array(
                Set(
                    state.namedPoseNames
                        .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
                        .filter { !$0.isEmpty }
                )
            ).sorted()
            safetyGate.knownPresets = Set(normalizedPresets.map { $0.lowercased() })
        }
        robotState.estopLatched = estopLatched
        robotState.enabled = !estopLatched
        robotState.connectionState = .connected

        if let openFraction = state.gripperOpenFraction {
            switch openFraction {
            case ..<0.1:
                robotState.gripperState = .closed
            case 0.9...:
                robotState.gripperState = .open
            default:
                robotState.gripperState = .moving
            }
        } else {
            robotState.gripperState = .unknown
        }
    }
}
