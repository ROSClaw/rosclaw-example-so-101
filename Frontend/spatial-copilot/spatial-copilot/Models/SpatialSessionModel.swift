import Foundation
import RosClawBridge
import simd

/// Immersive space state + spatial calibration + node invoke handling.
@MainActor
@Observable
final class SpatialSessionModel {

    let immersiveSpaceID = "spatial-workspace"

    enum ImmersiveSpaceState {
        case closed, inTransition, open
    }

    enum SpatialOnboardingStage: String, Sendable {
        case needsStylus
        case needsRobotState
        case placingBase
        case placingGripper
        case previewing
        case confirming
    }

    struct SpatialCalibrationPayload {
        let selectedSurfaceAnchorID: UUID
        let surfaceNormal: SIMD3<Float>
        let baseWorldPoint: SIMD3<Float>
        let gripperWorldPoint: SIMD3<Float>
        let robotToWorldTransform: simd_float4x4
        let worldToRobotTransform: simd_float4x4
    }

    struct SpatialPreviewState {
        let targetWorldPoint: SIMD3<Float>
        let requestedRobotPoint: SIMD3<Float>
        let clampedRobotPoint: SIMD3<Float>
        let insideWorkspace: Bool
        let previewJointPositions: [String: Double]
        let previewEndEffectorWorldPoint: SIMD3<Float>
        let positionErrorMeters: Double
    }

    var immersiveSpaceState: ImmersiveSpaceState = .closed
    var onboardingStage: SpatialOnboardingStage = .needsStylus
    var calibrationPayload: SpatialCalibrationPayload?
    var previewState: SpatialPreviewState?
    var pendingSpatialRobotGoal: SIMD3<Float>?
    var lastConfirmedSpatialRobotGoal: SIMD3<Float>?
    var pendingSurfaceAnchorID: UUID?
    var pendingSurfaceNormal: SIMD3<Float>?
    var pendingBaseWorldPoint: SIMD3<Float>?
    var pendingGripperWorldPoint: SIMD3<Float>?
    var pendingGoalWorldPoint: SIMD3<Float>?

    weak var connectionModel: RobotConnectionModel?
    weak var stateModel: RobotStateModel?
    weak var commandFlowModel: CommandFlowModel?

    var isCalibrated: Bool {
        calibrationPayload != nil
    }

    var spatialCalibrationWorldPosition: SIMD3<Float>? {
        calibrationPayload?.baseWorldPoint ?? pendingBaseWorldPoint
    }

    func synchronizeRequirements(stylusConnected: Bool, hasRobotToolPose: Bool) {
        if calibrationPayload != nil {
            if !stylusConnected {
                onboardingStage = .needsStylus
                return
            }
            if onboardingStage != .confirming {
                onboardingStage = .previewing
            }
            return
        }
        if !stylusConnected {
            onboardingStage = .needsStylus
            return
        }
        if !hasRobotToolPose {
            onboardingStage = .needsRobotState
            return
        }
        onboardingStage = pendingBaseWorldPoint == nil ? .placingBase : .placingGripper
    }

    func registerBasePlacement(
        surfaceAnchorID: UUID,
        surfaceNormal: SIMD3<Float>,
        worldPoint: SIMD3<Float>
    ) {
        pendingSurfaceAnchorID = surfaceAnchorID
        pendingSurfaceNormal = surfaceNormal
        pendingBaseWorldPoint = worldPoint
        previewState = nil
        pendingSpatialRobotGoal = nil
        pendingGoalWorldPoint = nil
        onboardingStage = .placingGripper
    }

    func completeCalibration(
        gripperWorldPoint: SIMD3<Float>,
        toolPose: RobotToolPose
    ) -> Bool {
        guard let selectedSurfaceAnchorID = pendingSurfaceAnchorID,
              let surfaceNormal = pendingSurfaceNormal,
              let baseWorldPoint = pendingBaseWorldPoint,
              let toolPosePosition = toolPose.position else {
            return false
        }

        let toolVector = SIMD3<Float>(
            Float(toolPosePosition.x),
            Float(toolPosePosition.y),
            Float(toolPosePosition.z)
        )
        guard let solved = SpatialTargetingMath.solveCalibration(
            baseWorldPoint: baseWorldPoint,
            gripperWorldPoint: gripperWorldPoint,
            toolPoseInRobotFrame: toolVector,
            surfaceNormal: surfaceNormal
        ) else {
            return false
        }

        pendingGripperWorldPoint = gripperWorldPoint
        calibrationPayload = SpatialCalibrationPayload(
            selectedSurfaceAnchorID: selectedSurfaceAnchorID,
            surfaceNormal: surfaceNormal,
            baseWorldPoint: baseWorldPoint,
            gripperWorldPoint: gripperWorldPoint,
            robotToWorldTransform: solved.robotToWorldTransform,
            worldToRobotTransform: solved.worldToRobotTransform
        )
        onboardingStage = .previewing
        return true
    }

    func resetCalibration() {
        calibrationPayload = nil
        previewState = nil
        pendingSpatialRobotGoal = nil
        lastConfirmedSpatialRobotGoal = nil
        pendingSurfaceAnchorID = nil
        pendingSurfaceNormal = nil
        pendingBaseWorldPoint = nil
        pendingGripperWorldPoint = nil
        pendingGoalWorldPoint = nil
        onboardingStage = .needsStylus
    }

    func setPreviewState(_ state: SpatialPreviewState?) {
        previewState = state
        if state == nil, onboardingStage != .confirming, calibrationPayload != nil {
            onboardingStage = .previewing
        }
    }

    func setPendingSpatialGoal(_ position: SIMD3<Float>, worldPoint: SIMD3<Float>) {
        pendingSpatialRobotGoal = position
        pendingGoalWorldPoint = worldPoint
        onboardingStage = .confirming
    }

    func clearPendingSpatialGoal() {
        pendingSpatialRobotGoal = nil
        pendingGoalWorldPoint = nil
        if calibrationPayload != nil {
            onboardingStage = .previewing
        }
    }

    func confirmSpatialGoal(_ position: SIMD3<Float>) {
        lastConfirmedSpatialRobotGoal = position
        pendingSpatialRobotGoal = nil
        pendingGoalWorldPoint = nil
        onboardingStage = .previewing
    }

    func currentWorkspaceSnapshot() -> GatewayWorkspaceSnapshot {
        GatewayWorkspaceSnapshot(
            stage: onboardingStage.rawValue,
            calibrationWorldPosition: calibrationPayload?.baseWorldPoint,
            calibrationSurfaceNormal: calibrationPayload?.surfaceNormal,
            calibrationGripperWorldPoint: calibrationPayload?.gripperWorldPoint,
            pendingRobotGoal: pendingSpatialRobotGoal,
            pendingRobotGoalWorldPoint: pendingGoalWorldPoint,
            lastConfirmedRobotGoal: lastConfirmedSpatialRobotGoal
        )
    }

    func sendCartesianGoal(x: Double, y: Double, z: Double) async {
        guard let stateModel, let connectionModel, let commandFlowModel else { return }
        let goal = SIMD3<Float>(Float(x), Float(y), Float(z))
        let target = RobotCartesianTarget(x: x, y: y, z: z, unit: .meters, referenceFrame: .baseAbsolute, source: .spatial)
        let command = RobotCommand.moveToCartesian(target: target)
        let safetyResult = stateModel.safetyGate.validate(command, robotState: stateModel.robotState)
        guard safetyResult.isAllowed else {
            commandFlowModel.appendTranscript(.error, safetyResult.blockReason ?? "Cartesian goal is not safe to execute.")
            commandFlowModel.appPhase = .idle
            return
        }
        confirmSpatialGoal(goal)
        commandFlowModel.appendTranscript(.parsedCommand, "Spatial target: \(command.displayName)")
        commandFlowModel.appPhase = .executing
        stateModel.robotState.actionStatus = .executing
        stateModel.robotState.mode = .executing
        do {
            let result = try await connectionModel.robotExecutor.execute(command: command, workspaceSnapshot: currentWorkspaceSnapshot())
            connectionModel.applyCommandResult(result)
            commandFlowModel.appendTranscript(.feedback, result.summary)
            commandFlowModel.appendCartesianResolutionTranscriptIfAvailable()
            commandFlowModel.appPhase = .idle
        } catch {
            stateModel.robotState.actionStatus = .failed
            stateModel.robotState.mode = .faulted
            stateModel.robotState.fault = error.localizedDescription
            commandFlowModel.appendTranscript(.error, "Cartesian goal failed: \(error.localizedDescription)")
            commandFlowModel.appPhase = .faulted
        }
    }

    // MARK: - Node invoke (vision.* commands from gateway)

    func handleNodeInvoke(_ request: BridgeInvokeRequest) -> BridgeInvokeResponse {
        switch request.command {
        case "vision.calibration.get":
            return invokeResponse(request.id, [
                "isCalibrated": calibrationPayload != nil,
                "onboardingStage": onboardingStage.rawValue,
                "calibrationWorldPosition": vectorDict(calibrationPayload?.baseWorldPoint) ?? NSNull(),
                "surfaceNormal": vectorDict(calibrationPayload?.surfaceNormal ?? pendingSurfaceNormal) ?? NSNull(),
                "gripperWorldPoint": vectorDict(calibrationPayload?.gripperWorldPoint ?? pendingGripperWorldPoint) ?? NSNull(),
                "lastConfirmedRobotGoal": vectorDict(lastConfirmedSpatialRobotGoal) ?? NSNull(),
            ])
        case "vision.goal.confirm":
            return invokeResponse(request.id, [
                "confirmed": lastConfirmedSpatialRobotGoal != nil || pendingSpatialRobotGoal != nil,
                "onboardingStage": onboardingStage.rawValue,
                "pendingRobotGoal": vectorDict(pendingSpatialRobotGoal) ?? NSNull(),
                "pendingRobotGoalWorldPoint": vectorDict(pendingGoalWorldPoint) ?? NSNull(),
                "lastConfirmedRobotGoal": vectorDict(lastConfirmedSpatialRobotGoal) ?? NSNull(),
            ])
        case "vision.anchor.snapshot":
            return invokeResponse(request.id, [
                "onboardingStage": onboardingStage.rawValue,
                "calibrationWorldPosition": vectorDict(calibrationPayload?.baseWorldPoint) ?? NSNull(),
                "surfaceNormal": vectorDict(calibrationPayload?.surfaceNormal ?? pendingSurfaceNormal) ?? NSNull(),
                "gripperWorldPoint": vectorDict(calibrationPayload?.gripperWorldPoint ?? pendingGripperWorldPoint) ?? NSNull(),
                "pendingRobotGoal": vectorDict(pendingSpatialRobotGoal) ?? NSNull(),
                "pendingRobotGoalWorldPoint": vectorDict(pendingGoalWorldPoint) ?? NSNull(),
                "lastConfirmedRobotGoal": vectorDict(lastConfirmedSpatialRobotGoal) ?? NSNull(),
            ])
        default:
            return BridgeInvokeResponse(id: request.id, ok: false,
                error: OpenClawNodeError(code: .invalidRequest, message: "Unsupported node command: \(request.command)"))
        }
    }

    private func invokeResponse(_ id: String, _ payload: [String: Any]) -> BridgeInvokeResponse {
        let json: String? = {
            guard JSONSerialization.isValidJSONObject(payload),
                  let data = try? JSONSerialization.data(withJSONObject: payload),
                  let s = String(data: data, encoding: .utf8) else { return nil }
            return s
        }()
        return BridgeInvokeResponse(id: id, ok: true, payloadJSON: json)
    }

    private func vectorDict(_ v: SIMD3<Float>?) -> [String: Double]? {
        guard let v else { return nil }
        return ["x": Double(v.x), "y": Double(v.y), "z": Double(v.z)]
    }
}
