import SwiftUI
import RosClawBridge

/// Settings presented as a sheet from the copilot panel.
struct SettingsSheet: View {
    @Environment(RobotConnectionModel.self) private var connection
    @Environment(RobotStateModel.self) private var state
    @Environment(CommandFlowModel.self) private var commandFlow
    @Environment(SpatialStylusModel.self) private var stylus
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        @Bindable var flow = commandFlow
        NavigationStack {
            Form {
                Section("Remote Agent") {
                    LabeledContent("Endpoint") { Text(connection.remoteEndpointDescription).font(.body.weight(.medium)).textSelection(.enabled) }
                    LabeledContent("Boot State") { Text(connection.remoteBootState.rawValue).font(.body.weight(.bold)).foregroundStyle(bootColor) }
                    LabeledContent("Agent") { Text(connection.remoteAgentID ?? "Not resolved").font(.body.weight(.medium)).textSelection(.enabled) }
                    LabeledContent("Session") { Text(connection.remoteSessionKey ?? "Not resolved").font(.body.weight(.medium)).textSelection(.enabled) }
                    LabeledContent("Node") { Text(connection.remoteNodeID ?? "Not registered").font(.body.weight(.medium)).textSelection(.enabled) }
                    LabeledContent("Auth") { Text(connection.remoteAuthConfigured ? "Configured" : "Not set").font(.body.weight(.medium)) }
                    LabeledContent("TLS") { Text(connection.remoteTLSConfigured ? "wss" : "ws").font(.body.weight(.bold)) }
                    if let poseFlow = connection.remotePoseFlowSummary {
                        LabeledContent("Pose Flow") {
                            Text(poseFlow)
                                .font(.callout.weight(.medium))
                                .multilineTextAlignment(.trailing)
                                .foregroundStyle(.secondary)
                        }
                    }
                    if let stateSummary = connection.remoteStateSummary {
                        LabeledContent("State Sync") {
                            Text(stateSummary)
                                .font(.callout.weight(.medium))
                                .multilineTextAlignment(.trailing)
                                .foregroundStyle(.secondary)
                        }
                    }
                    if let ts = connection.lastSnapshotTimestamp { LabeledContent("Snapshot") { Text(ts).font(.body.weight(.medium)).textSelection(.enabled) } }
                    if let err = connection.remoteConfigurationError { Text(err).font(.callout.weight(.medium)).foregroundStyle(.red) }
                    HStack(spacing: 16) {
                        Button("Reconnect") { Task { await connection.bootstrapRemoteAgent(forceRecreate: true) } }
                            .buttonStyle(.borderedProminent).hoverEffect()
                        Button("Refresh") { Task { await connection.refreshRobotSnapshot() } }
                            .buttonStyle(.bordered).hoverEffect()
                    }.padding(.top, 8)
                }
                Section("Workspace") {
                    LabeledContent("Bounds") {
                        let wb = connection.workspaceBounds
                        Text("x [\(wb.xMin), \(wb.xMax)]\ny [\(wb.yMin), \(wb.yMax)]\nz [\(wb.zMin), \(wb.zMax)]")
                            .font(.callout.weight(.medium)).multilineTextAlignment(.trailing)
                    }
                    LabeledContent("Presets") { Text(connection.supportedPresetSummary.isEmpty ? "None" : connection.supportedPresetSummary.joined(separator: ", ")).font(.callout.weight(.medium)).foregroundStyle(.secondary) }
                    LabeledContent("Pose") { Text(state.currentNamedPose ?? "Unlabeled").font(.callout.weight(.semibold)).foregroundStyle(.secondary) }
                    LabeledContent("Tool Pose") {
                        Text(state.toolPoseStatus?.stage.replacingOccurrences(of: "_", with: " ").capitalized ?? "Unknown")
                            .font(.callout.weight(.semibold))
                            .foregroundStyle(state.robotState.toolPose?.position != nil ? .green : .orange)
                    }
                    LabeledContent("Pose Details") {
                        Text(state.toolPoseStatusSummary)
                            .font(.callout.weight(.medium))
                            .multilineTextAlignment(.trailing)
                            .foregroundStyle(.secondary)
                    }
                    if let status = state.toolPoseStatus,
                       !status.missingJointNames.isEmpty {
                        LabeledContent("Missing Joints") {
                            Text(status.missingJointNames.joined(separator: ", "))
                                .font(.callout.monospaced())
                                .multilineTextAlignment(.trailing)
                                .foregroundStyle(.secondary)
                        }
                    }
                    LabeledContent("Gripper Joint") {
                        Text(
                            state.robotState.gripperPosition.map { String(format: "%.4f", $0) }
                                ?? "Unavailable"
                        )
                        .font(.callout.weight(.medium))
                        .foregroundStyle(.secondary)
                    }
                    LabeledContent("Gripper Open") {
                        Text(
                            state.robotState.gripperOpenFraction.map { String(format: "%.1f%%", $0 * 100.0) }
                                ?? "Unavailable"
                        )
                        .font(.callout.weight(.medium))
                        .foregroundStyle(.secondary)
                    }
                }
                Section("Stylus") {
                    LabeledContent("Connected") { Text(stylus.isConnected ? "Yes" : "No").font(.body.weight(.semibold)).foregroundStyle(stylus.isConnected ? .green : .secondary) }
                    LabeledContent("Tracking") { Text(stylus.isTracking ? "Active" : "Idle").font(.body.weight(.medium)) }
                }
                Section("AI Model") {
                    if commandFlow.availableProviderIDs.isEmpty {
                        Text("No providers configured.").font(.callout).foregroundStyle(.secondary)
                    } else {
                        Picker("Provider", selection: $flow.selectedProviderID) {
                            ForEach(commandFlow.availableProviderIDs, id: \.self) { id in Text(id.capitalized).tag(id) }
                        }
                    }
                }
                Section("Safety") {
                    LabeledContent("Known Presets") { Text(state.safetyGate.knownPresets.sorted().joined(separator: ", ")).font(.callout.weight(.medium)).foregroundStyle(.secondary) }
                }
            }
            .formStyle(.grouped)
            .scrollContentBackground(.hidden)
            .navigationTitle("Settings")
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
        }
    }

    private var bootColor: Color {
        switch connection.remoteBootState {
        case .ready: return .green
        case .faulted: return .red
        default: return .orange
        }
    }
}
