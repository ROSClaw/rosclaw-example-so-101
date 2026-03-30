import Foundation
import RosClawBridge
import simd

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
        let derivedGripperOpenFraction = resolvedGripperOpenFraction(from: state)
        let synthesizedToolPose = synthesizedToolPose(from: state)
        robotState.jointOrder = state.jointOrder
        robotState.jointPositions = state.jointPositions
        robotState.gripperPosition = state.gripperPosition
        robotState.gripperOpenFraction = derivedGripperOpenFraction
        robotState.toolPose = state.toolPose ?? synthesizedToolPose
        toolPoseStatus = resolvedToolPoseStatus(from: state, synthesizedToolPose: synthesizedToolPose)
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

        if let openFraction = derivedGripperOpenFraction {
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

    private func resolvedGripperOpenFraction(from state: GatewayFollowerState) -> Double? {
        if let fraction = state.gripperOpenFraction {
            return min(max(fraction, 0.0), 1.0)
        }

        let rawPosition = state.gripperPosition ?? state.jointPositions["gripper"]
        guard let rawPosition else { return nil }
        return min(max(rawPosition, 0.0), 1.0)
    }

    private func synthesizedToolPose(from state: GatewayFollowerState) -> RobotToolPose? {
        guard state.toolPose == nil else { return nil }

        let missingJoints = SO101ArmJoint.allCases.filter { state.jointPositions[$0.rawValue] == nil }
        guard missingJoints.isEmpty else { return nil }

        let jointAngles = SO101Kinematics.clampToJointLimits(
            SO101JointAngles.seeded(from: state.jointPositions)
        )
        let forwardKinematics = SO101Kinematics.forwardKinematics(jointAngles: jointAngles)
        let position = forwardKinematics.endEffectorPosition
        let orientation = simd_quatd(forwardKinematics.endEffectorTransform.upperLeft3x3)

        return RobotToolPose(
            frameID: FollowerTopics.baseFrame,
            position: RobotVector3(x: position.x, y: position.y, z: position.z),
            orientation: RobotQuaternion(
                x: orientation.vector.x,
                y: orientation.vector.y,
                z: orientation.vector.z,
                w: orientation.vector.w
            )
        )
    }

    private func resolvedToolPoseStatus(
        from state: GatewayFollowerState,
        synthesizedToolPose: RobotToolPose?
    ) -> GatewayToolPoseStatus? {
        if let synthesizedToolPose {
            let missingJoints = SO101ArmJoint.allCases
                .map(\.rawValue)
                .filter { state.jointPositions[$0] == nil }
            let position = synthesizedToolPose.position
            let summary: String
            if let position {
                summary = String(
                    format: "Backend tool pose is unavailable; using local SO-101 forward kinematics at x=%.3f m, y=%.3f m, z=%.3f m.",
                    position.x,
                    position.y,
                    position.z
                )
            } else {
                summary = "Backend tool pose is unavailable; using local SO-101 forward kinematics."
            }
            return GatewayToolPoseStatus(
                stage: "local_fk_fallback",
                summary: summary,
                jointStateReceived: !state.jointOrder.isEmpty,
                jointCount: state.jointPositions.count,
                robotDescriptionReceived: state.toolPoseStatus?.robotDescriptionReceived ?? false,
                kinematicsReady: true,
                missingJointNames: missingJoints,
                baseFrame: FollowerTopics.baseFrame,
                toolFrame: FollowerTopics.toolFrame,
                updatedAt: ISO8601DateFormatter().string(from: Date())
            )
        }
        return state.toolPoseStatus
    }
}
