import Foundation
import OpenClawKit
import OpenClawProtocol
import RosClawBridge

/// Gateway connection lifecycle — bootstrap, reconnect, snapshot sync.
@MainActor
@Observable
final class RobotConnectionModel {

    var remoteBootState: GatewayRobotBootState = .readingConfig
    var remoteEndpointDescription: String = "Not configured"
    var remoteAuthConfigured = false
    var remoteTLSConfigured = false
    var remoteConfigurationError: String?
    var remoteStateSummary: String?
    var remotePoseFlowSummary: String?
    var lastSnapshotTimestamp: String?
    var operationRegistrySummary: [String] = []
    var remoteAgentID: String?
    var remoteSessionKey: String?
    var remoteNodeID: String?
    var supportedPresetSummary: [String] = []
    var workspaceBounds: WorkspaceBounds = .followerDefault

    let robotExecutor = GatewayRobotExecutor()
    private let isoFormatter = ISO8601DateFormatter()
    private var hasBootstrapped = false

    /// Weak references to sibling models for cross-model updates.
    weak var stateModel: RobotStateModel?
    weak var spatialModel: SpatialSessionModel?

    func bootstrapIfNeeded() async {
        guard !hasBootstrapped else { return }
        hasBootstrapped = true
        if remoteBootState != .ready {
            await bootstrapRemoteAgent(forceRecreate: false)
        }
    }

    func bootstrapRemoteAgent(forceRecreate: Bool = false) async {
        do {
            let endpoint = try GatewayRobotEndpointConfiguration.from(
                environment: ProcessInfo.processInfo.environment
            )
            remoteBootState = .readingConfig
            remoteConfigurationError = nil
            remoteEndpointDescription = endpoint.gatewayURL.absoluteString
            remoteAuthConfigured =
                endpoint.authToken != nil ||
                endpoint.bootstrapToken != nil ||
                endpoint.password != nil
            remoteTLSConfigured = endpoint.gatewayURL.scheme?.lowercased() == "wss"
            remoteStateSummary = nil

            if forceRecreate { await robotExecutor.disconnect() }

            stateModel?.robotState.connectionState = .connecting
            stateModel?.robotState.enabled = false
            stateModel?.toolPoseStatus = nil
            remoteBootState = .connectingOperator
            remotePoseFlowSummary = "Sending bootstrap request to ROSClaw…"

            let snapshot = try await robotExecutor.bootstrap(
                configuration: endpoint,
                snapshotUpdateHandler: { [weak self] snapshot in
                    guard let self else { return }
                    await MainActor.run { self.applySnapshot(snapshot) }
                },
                nodeInvokeHandler: { [weak self] request in
                    guard let self else {
                        return BridgeInvokeResponse(
                            id: request.id, ok: false,
                            error: OpenClawNodeError(code: .unavailable, message: "visionOS app unavailable"))
                    }
                    return await MainActor.run {
                        self.spatialModel?.handleNodeInvoke(request) ?? BridgeInvokeResponse(
                            id: request.id, ok: false,
                            error: OpenClawNodeError(code: .unavailable, message: "Spatial model unavailable"))
                    }
                }
            )
            applySnapshot(snapshot)
            remoteBootState = .ready
        } catch {
            remoteBootState = .faulted
            remoteConfigurationError = error.localizedDescription
            remoteStateSummary = nil
            remotePoseFlowSummary = error.localizedDescription
            stateModel?.robotState.connectionState = .disconnected
            stateModel?.robotState.enabled = false
            stateModel?.robotState.mode = .faulted
            stateModel?.robotState.fault = error.localizedDescription
        }
    }

    func refreshRobotSnapshot() async {
        do {
            remoteBootState = .syncingState
            remotePoseFlowSummary = "Requesting the latest robot pose from ROSClaw…"
            let snapshot = try await robotExecutor.refreshSnapshot()
            applySnapshot(snapshot)
            remoteBootState = .ready
        } catch {
            remoteBootState = .faulted
            remotePoseFlowSummary = error.localizedDescription
            stateModel?.robotState.mode = .faulted
            stateModel?.robotState.fault = error.localizedDescription
        }
    }
// MARK: - Snapshot application

    func applySnapshot(_ snapshot: GatewayRobotSnapshot) {
        remoteAgentID = snapshot.executorAgentID
        remoteSessionKey = snapshot.executorSessionKey
        remoteNodeID = snapshot.nodeID
        remoteStateSummary = snapshot.stateSummary
        operationRegistrySummary = snapshot.capabilities.supportedCommands
        supportedPresetSummary = snapshot.capabilities.supportedPresets
        workspaceBounds = snapshot.capabilities.workspaceBounds

        guard let stateModel else { return }
        stateModel.safetyGate.knownPresets = Set(snapshot.capabilities.supportedPresets.map { $0.lowercased() })
        stateModel.safetyGate.workspaceBounds = snapshot.capabilities.workspaceBounds
        stateModel.robotState.connectionState = .connected
        stateModel.robotState.estopLatched = snapshot.estopLatched

        if let state = snapshot.state {
            remotePoseFlowSummary = state.toolPoseStatus?.summary
                ?? (state.toolPose?.position != nil
                    ? "Pose data received from ROSClaw and processed by the app."
                    : snapshot.stateSummary ?? "Waiting for pose data from ROSClaw.")
            stateModel.robotState.enabled = !snapshot.estopLatched
            stateModel.robotState.mode = snapshot.estopLatched ? .faulted : .idle
            stateModel.robotState.fault = snapshot.estopLatched ? "Emergency stop latched" : nil
            stateModel.applyFollowerState(state, estopLatched: snapshot.estopLatched)
            // State-level values override capability defaults
            if let stateWorkspaceBounds = state.workspaceBounds {
                workspaceBounds = stateWorkspaceBounds
            }
            if !state.namedPoseNames.isEmpty {
                let merged = Array(
                    Set((supportedPresetSummary + state.namedPoseNames)
                        .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
                        .filter { !$0.isEmpty })
                ).sorted()
                supportedPresetSummary = merged
                stateModel.safetyGate.knownPresets = Set(merged.map { $0.lowercased() })
            }
        } else if snapshot.estopLatched {
            remotePoseFlowSummary = snapshot.stateSummary ?? "Emergency stop latched before pose data was available."
            stateModel.robotState.enabled = false
            stateModel.robotState.mode = .faulted
            stateModel.robotState.fault = "Emergency stop latched"
            stateModel.currentNamedPose = nil
            stateModel.lastResolvedCartesianGoal = nil
            stateModel.toolPoseStatus = nil
        } else {
            remotePoseFlowSummary = snapshot.stateSummary ?? "Waiting for pose data from ROSClaw."
            stateModel.robotState.enabled = false
            stateModel.robotState.mode = .unknown
            stateModel.robotState.fault = snapshot.stateSummary ?? "Robot state unavailable."
            stateModel.currentNamedPose = nil
            stateModel.lastResolvedCartesianGoal = nil
            stateModel.toolPoseStatus = nil
        }

        if !snapshot.capabilities.supportedPresets.isEmpty {
            supportedPresetSummary = Array(
                Set(snapshot.capabilities.supportedPresets
                    .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
                    .filter { !$0.isEmpty })
            ).sorted()
        }
        lastSnapshotTimestamp = isoFormatter.string(from: Date())
    }

    func applyCommandResult(_ result: GatewayRobotCommandResult) {
        guard let stateModel else { return }
        if let state = result.state {
            remoteStateSummary = nil
            remotePoseFlowSummary = state.toolPoseStatus?.summary
                ?? (state.toolPose?.position != nil
                    ? "Pose data received from ROSClaw and processed by the app."
                    : "Command completed, but tool pose is still unavailable.")
            stateModel.applyFollowerState(state, estopLatched: result.estopLatched)
        } else {
            stateModel.robotState.estopLatched = result.estopLatched
            stateModel.robotState.enabled = !result.estopLatched
            remotePoseFlowSummary = result.summary
            stateModel.toolPoseStatus = nil
        }
        if result.ok {
            stateModel.robotState.actionStatus = result.estopLatched ? .cancelled : .succeeded
            stateModel.robotState.mode = result.estopLatched ? .faulted : .idle
            stateModel.robotState.fault = result.estopLatched ? result.summary : nil
        } else {
            stateModel.robotState.actionStatus = .failed
            stateModel.robotState.mode = .faulted
            stateModel.robotState.fault = result.summary
        }
        lastSnapshotTimestamp = isoFormatter.string(from: Date())
    }

    func supportsCommand(_ name: String) -> Bool {
        operationRegistrySummary.contains(name)
    }

    var supportsHomeCommand: Bool {
        supportedPresetSummary.contains { $0.caseInsensitiveCompare("home") == .orderedSame }
    }
}
