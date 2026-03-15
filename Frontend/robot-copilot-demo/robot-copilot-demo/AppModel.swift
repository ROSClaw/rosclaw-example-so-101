import SwiftUI
import OpenClawAgents
import OpenClawCore
import OpenClawModels
import OpenClawKit
import OpenClawProtocol
import RosClawBridge

enum AppPhase: String, Sendable {
    case idle = "Idle"
    case previewing = "Previewing"
    case awaitingConfirmation = "Awaiting Confirmation"
    case executing = "Executing"
    case faulted = "Faulted"
}

struct TranscriptEntry: Identifiable, Sendable {
    enum Kind: Sendable {
        case userInput
        case parsedCommand
        case feedback
        case error
        case system
    }

    let id = UUID()
    let kind: Kind
    let text: String
    let timestamp: Date = Date()
}

@MainActor
@Observable
class AppModel {
    let immersiveSpaceID = "spatial-alignment"

    enum ImmersiveSpaceState {
        case closed
        case inTransition
        case open
    }

    var immersiveSpaceState: ImmersiveSpaceState = .closed

    var robotState: RobotState = RobotState()
    var remoteBootState: GatewayRobotBootState = .readingConfig
    var remoteEndpointDescription: String = "Not configured"
    var remoteAuthConfigured = false
    var remoteTLSConfigured = false
    var remoteConfigurationError: String?
    var remoteStateSummary: String?
    var lastSnapshotTimestamp: String?
    var operationRegistrySummary: [String] = []
    var remoteAgentID: String?
    var remoteSessionKey: String?
    var remoteNodeID: String?
    var supportedPresetSummary: [String] = []
    var workspaceBounds: WorkspaceBounds = .followerDefault
    var currentNamedPose: String?
    var lastResolvedCartesianGoal: GatewayResolvedCartesianGoal?

    var appPhase: AppPhase = .idle
    var pendingCommand: RobotCommand?
    var pendingPreview: CommandPreview?
    var transcript: [TranscriptEntry] = []

    let diagnosticsPipeline: RuntimeDiagnosticsPipeline
    let agentRuntime: EmbeddedAgentRuntime

    var availableProviderIDs: [String] = []
    var selectedProviderID: String = ""

    var safetyGate: SafetyGate = SafetyGate()

    var spatialCalibrationWorldPosition: SIMD3<Float>?
    var pendingSpatialRobotGoal: SIMD3<Float>?
    var lastConfirmedSpatialRobotGoal: SIMD3<Float>?

    let robotExecutor = GatewayRobotExecutor()
    private var hasSetupModelProviders = false
    private let isoFormatter = ISO8601DateFormatter()

    init() {
        let sdk = OpenClawSDK.shared
        self.diagnosticsPipeline = sdk.makeDiagnosticsPipeline(eventLimit: 500)
        self.agentRuntime = EmbeddedAgentRuntime()
    }

    func bootstrapIfNeeded() async {
        if !hasSetupModelProviders {
            await setupModelProviders()
            hasSetupModelProviders = true
        }

        if remoteBootState != .ready {
            await bootstrapRemoteAgent(forceRecreate: false)
        }
    }

    func bootstrapRemoteAgent(forceRecreate: Bool = false) async {
        do {
            let endpoint = try GatewayRobotEndpointConfiguration.from(environment: ProcessInfo.processInfo.environment)
            remoteBootState = .readingConfig
            remoteConfigurationError = nil
            remoteEndpointDescription = endpoint.gatewayURL.absoluteString
            remoteAuthConfigured =
                endpoint.authToken != nil ||
                endpoint.bootstrapToken != nil ||
                endpoint.password != nil
            remoteTLSConfigured = endpoint.gatewayURL.scheme?.lowercased() == "wss"

            if forceRecreate {
                await robotExecutor.disconnect()
            }

            robotState.connectionState = .connecting
            robotState.enabled = false
            remoteBootState = .connectingOperator
            appendTranscript(.system, "Connecting to OpenClaw Gateway at \(endpoint.gatewayURL.absoluteString)")

            let snapshot = try await robotExecutor.bootstrap(
                configuration: endpoint,
                snapshotUpdateHandler: { [weak self] snapshot in
                    guard let self else { return }
                    await MainActor.run {
                        self.applySnapshot(snapshot)
                    }
                },
                nodeInvokeHandler: { [weak self] request in
                    guard let self else {
                        return BridgeInvokeResponse(
                            id: request.id,
                            ok: false,
                            error: OpenClawNodeError(code: .unavailable, message: "visionOS app unavailable"))
                    }
                    return await MainActor.run {
                        self.handleNodeInvoke(request)
                    }
                }
            )

            applySnapshot(snapshot)
            remoteBootState = .ready
            if let stateSummary = snapshot.stateSummary {
                appendTranscript(.system, "Remote ROSClaw agent connected: \(snapshot.executorAgentID) (\(stateSummary))")
            } else {
                appendTranscript(.system, "Remote ROSClaw agent ready: \(snapshot.executorAgentID)")
            }
        } catch {
            remoteBootState = .faulted
            remoteConfigurationError = error.localizedDescription
            remoteStateSummary = nil
            robotState.connectionState = .disconnected
            robotState.enabled = false
            robotState.mode = .faulted
            robotState.fault = error.localizedDescription
            appendTranscript(.error, "Gateway bootstrap failed: \(error.localizedDescription)")
        }
    }

    func refreshRobotSnapshot() async {
        do {
            remoteBootState = .syncingState
            let snapshot = try await robotExecutor.refreshSnapshot()
            applySnapshot(snapshot)
            remoteBootState = .ready
            appendTranscript(.system, snapshot.stateSummary ?? "Robot state synchronized")
        } catch {
            remoteBootState = .faulted
            robotState.mode = .faulted
            robotState.fault = error.localizedDescription
            appendTranscript(.error, "State sync failed: \(error.localizedDescription)")
        }
    }

    private func setupModelProviders() async {
        let env = ProcessInfo.processInfo.environment
        var ids: [String] = []

        if let apiKey = env["OPENAI_API_KEY"], !apiKey.isEmpty {
            let config = OpenAIModelConfig(
                enabled: true,
                modelID: "gpt-5.4",
                apiKey: apiKey
            )
            let provider = OpenAIModelProvider(configuration: config)
            await agentRuntime.registerModelProvider(provider)
            ids.append(OpenAIModelProvider.providerID)
        }

        let foundationAvailability = FoundationModelsProvider.runtimeAvailability()
        if foundationAvailability.isAvailable {
            let provider = FoundationModelsProvider()
            await agentRuntime.registerModelProvider(provider)
            ids.append(FoundationModelsProvider.providerID)
        }

        availableProviderIDs = ids
        if let first = ids.first {
            selectedProviderID = first
            try? await agentRuntime.setDefaultModelProviderID(first)
        }
    }

    func applySnapshot(_ snapshot: GatewayRobotSnapshot) {
        self.remoteAgentID = snapshot.executorAgentID
        self.remoteSessionKey = snapshot.executorSessionKey
        self.remoteNodeID = snapshot.nodeID
        self.remoteStateSummary = snapshot.stateSummary
        self.operationRegistrySummary = snapshot.capabilities.supportedCommands
        self.supportedPresetSummary = snapshot.capabilities.supportedPresets
        self.workspaceBounds = snapshot.capabilities.workspaceBounds
        self.safetyGate.knownPresets = Set(snapshot.capabilities.supportedPresets.map { $0.lowercased() })
        self.safetyGate.workspaceBounds = snapshot.capabilities.workspaceBounds

        robotState.connectionState = .connected
        robotState.estopLatched = snapshot.estopLatched
        if let state = snapshot.state {
            robotState.enabled = !snapshot.estopLatched
            robotState.mode = snapshot.estopLatched ? .faulted : .idle
            robotState.fault = snapshot.estopLatched ? "Emergency stop latched" : nil
            applyFollowerState(state, estopLatched: snapshot.estopLatched)
        } else if snapshot.estopLatched {
            robotState.enabled = false
            robotState.mode = .faulted
            robotState.fault = "Emergency stop latched"
            currentNamedPose = nil
            lastResolvedCartesianGoal = nil
        } else {
            robotState.enabled = false
            robotState.mode = .unknown
            robotState.fault = snapshot.stateSummary ?? "Robot state unavailable."
            currentNamedPose = nil
            lastResolvedCartesianGoal = nil
        }
        lastSnapshotTimestamp = isoFormatter.string(from: Date())
    }

    func applyCommandResult(_ result: GatewayRobotCommandResult) {
        if let state = result.state {
            remoteStateSummary = nil
            applyFollowerState(state, estopLatched: result.estopLatched)
        } else {
            robotState.estopLatched = result.estopLatched
            robotState.enabled = !result.estopLatched
        }

        if result.ok {
            robotState.actionStatus = result.estopLatched ? .cancelled : .succeeded
            robotState.mode = result.estopLatched ? .faulted : .idle
            robotState.fault = result.estopLatched ? result.summary : nil
        } else {
            robotState.actionStatus = .failed
            robotState.mode = .faulted
            robotState.fault = result.summary
        }

        lastSnapshotTimestamp = isoFormatter.string(from: Date())
    }

    private func applyFollowerState(_ state: GatewayFollowerState, estopLatched: Bool) {
        robotState.jointOrder = state.jointOrder
        robotState.jointPositions = state.jointPositions
        robotState.gripperPosition = state.gripperPosition
        robotState.gripperOpenFraction = state.gripperOpenFraction
        robotState.toolPose = state.toolPose
        currentNamedPose = state.currentNamedPose
        lastResolvedCartesianGoal = state.lastResolvedCartesianGoal

        if let stateWorkspaceBounds = state.workspaceBounds {
            workspaceBounds = stateWorkspaceBounds
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
            supportedPresetSummary = normalizedPresets
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

    func registerSpatialCalibration(worldPosition: SIMD3<Float>) {
        spatialCalibrationWorldPosition = worldPosition
    }

    func setPendingSpatialGoal(_ position: SIMD3<Float>) {
        pendingSpatialRobotGoal = position
    }

    func clearPendingSpatialGoal() {
        pendingSpatialRobotGoal = nil
    }

    func confirmSpatialGoal(_ position: SIMD3<Float>) {
        lastConfirmedSpatialRobotGoal = position
        pendingSpatialRobotGoal = nil
    }

    func currentWorkspaceSnapshot() -> GatewayWorkspaceSnapshot {
        GatewayWorkspaceSnapshot(
            calibrationWorldPosition: spatialCalibrationWorldPosition,
            pendingRobotGoal: pendingSpatialRobotGoal,
            lastConfirmedRobotGoal: lastConfirmedSpatialRobotGoal
        )
    }

    private func handleNodeInvoke(_ request: BridgeInvokeRequest) -> BridgeInvokeResponse {
        switch request.command {
        case "vision.calibration.get":
            let payload: [String: Any] = [
                "isCalibrated": spatialCalibrationWorldPosition != nil,
                "calibrationWorldPosition": vectorDictionary(spatialCalibrationWorldPosition) ?? NSNull(),
                "lastConfirmedRobotGoal": vectorDictionary(lastConfirmedSpatialRobotGoal) ?? NSNull(),
            ]
            return BridgeInvokeResponse(id: request.id, ok: true, payloadJSON: jsonString(payload))

        case "vision.goal.confirm":
            let payload: [String: Any] = [
                "confirmed": lastConfirmedSpatialRobotGoal != nil || pendingSpatialRobotGoal != nil,
                "pendingRobotGoal": vectorDictionary(pendingSpatialRobotGoal) ?? NSNull(),
                "lastConfirmedRobotGoal": vectorDictionary(lastConfirmedSpatialRobotGoal) ?? NSNull(),
            ]
            return BridgeInvokeResponse(id: request.id, ok: true, payloadJSON: jsonString(payload))

        case "vision.anchor.snapshot":
            let payload: [String: Any] = [
                "calibrationWorldPosition": vectorDictionary(spatialCalibrationWorldPosition) ?? NSNull(),
                "pendingRobotGoal": vectorDictionary(pendingSpatialRobotGoal) ?? NSNull(),
                "lastConfirmedRobotGoal": vectorDictionary(lastConfirmedSpatialRobotGoal) ?? NSNull(),
            ]
            return BridgeInvokeResponse(id: request.id, ok: true, payloadJSON: jsonString(payload))

        default:
            return BridgeInvokeResponse(
                id: request.id,
                ok: false,
                error: OpenClawNodeError(
                    code: .invalidRequest,
                    message: "Unsupported node command: \(request.command)")
            )
        }
    }

    private func vectorDictionary(_ vector: SIMD3<Float>?) -> [String: Double]? {
        guard let vector else { return nil }
        return [
            "x": Double(vector.x),
            "y": Double(vector.y),
            "z": Double(vector.z),
        ]
    }

    private func jsonString(_ object: [String: Any]) -> String? {
        guard JSONSerialization.isValidJSONObject(object),
              let data = try? JSONSerialization.data(withJSONObject: object),
              let string = String(data: data, encoding: .utf8)
        else {
            return nil
        }
        return string
    }

    func appendTranscript(_ kind: TranscriptEntry.Kind, _ text: String) {
        transcript.append(TranscriptEntry(kind: kind, text: text))
        if transcript.count > 200 {
            transcript.removeFirst(transcript.count - 200)
        }
    }
}
