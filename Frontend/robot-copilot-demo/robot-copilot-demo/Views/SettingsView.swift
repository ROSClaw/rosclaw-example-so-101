import SwiftUI
import RosClawBridge

struct SettingsView: View {
    @Environment(AppModel.self) private var appModel
    @Environment(SpatialStylusModel.self) private var stylusModel

    var body: some View {
        @Bindable var model = appModel
        Form {
            Section("Remote Agent") {
                LabeledContent("Endpoint") {
                    Text(appModel.remoteEndpointDescription)
                        .multilineTextAlignment(.trailing)
                        .textSelection(.enabled)
                }
                LabeledContent("Boot State") {
                    Text(appModel.remoteBootState.rawValue)
                        .foregroundStyle(remoteBootStateColor)
                }
                LabeledContent("Executor Agent") {
                    Text(appModel.remoteAgentID ?? "Not resolved")
                        .multilineTextAlignment(.trailing)
                        .textSelection(.enabled)
                }
                LabeledContent("Executor Session") {
                    Text(appModel.remoteSessionKey ?? "Not resolved")
                        .multilineTextAlignment(.trailing)
                        .textSelection(.enabled)
                }
                LabeledContent("Node ID") {
                    Text(appModel.remoteNodeID ?? "Not registered")
                        .multilineTextAlignment(.trailing)
                        .textSelection(.enabled)
                }
                LabeledContent("Auth Token") {
                    Text(appModel.remoteAuthConfigured ? "Configured" : "Not set")
                }
                LabeledContent("TLS") {
                    Text(appModel.remoteTLSConfigured ? "wss" : "ws")
                }
                if let timestamp = appModel.lastSnapshotTimestamp {
                    LabeledContent("Last Snapshot") {
                        Text(timestamp)
                            .multilineTextAlignment(.trailing)
                            .textSelection(.enabled)
                    }
                }
                if let stateSummary = appModel.remoteStateSummary {
                    LabeledContent("Robot State") {
                        Text(stateSummary)
                            .multilineTextAlignment(.trailing)
                            .foregroundStyle(.orange)
                    }
                }
                if let error = appModel.remoteConfigurationError {
                    Text(error)
                        .font(.caption)
                        .foregroundStyle(.red)
                }
                HStack {
                    Button("Reconnect") {
                        Task { await appModel.bootstrapRemoteAgent(forceRecreate: true) }
                    }
                    .buttonStyle(.borderedProminent)

                    Button("Refresh Snapshot") {
                        Task { await appModel.refreshRobotSnapshot() }
                    }
                    .buttonStyle(.bordered)
                    .disabled(appModel.remoteBootState == .faulted && appModel.remoteConfigurationError != nil)
                }
            }

            Section("Operation Registry") {
                if appModel.operationRegistrySummary.isEmpty {
                    Text("No backend-backed command surface has been loaded yet.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                } else {
                    Text(appModel.operationRegistrySummary.joined(separator: ", "))
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }

            Section("Workspace") {
                LabeledContent("Bounds") {
                    Text(
                        "x [\(appModel.workspaceBounds.xMin), \(appModel.workspaceBounds.xMax)]\n"
                            + "y [\(appModel.workspaceBounds.yMin), \(appModel.workspaceBounds.yMax)]\n"
                            + "z [\(appModel.workspaceBounds.zMin), \(appModel.workspaceBounds.zMax)]"
                    )
                    .font(.caption)
                    .multilineTextAlignment(.trailing)
                }
                LabeledContent("Presets") {
                    Text(appModel.supportedPresetSummary.isEmpty ? "None" : appModel.supportedPresetSummary.joined(separator: ", "))
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                LabeledContent("Current Pose") {
                    Text(appModel.currentNamedPose ?? "Unlabeled")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }

            Section("Stylus") {
                LabeledContent("Connected") {
                    Text(stylusModel.isConnected ? "Yes" : "No")
                }
                LabeledContent("Tracking") {
                    Text(stylusModel.isTracking ? "Active" : "Idle")
                }
            }

            if let resolution = appModel.lastResolvedCartesianGoal {
                Section("Last Resolved Goal") {
                    LabeledContent("Requested") {
                        Text(positionSummary(resolution.requestedPose))
                            .font(.caption)
                            .multilineTextAlignment(.trailing)
                    }
                    LabeledContent("Resolved") {
                        Text(positionSummary(resolution.resolvedPose))
                            .font(.caption)
                            .multilineTextAlignment(.trailing)
                    }
                    LabeledContent("Reason") {
                        Text(resolution.reason)
                            .font(.caption)
                            .multilineTextAlignment(.trailing)
                    }
                }
            }

            Section("AI Model") {
                if appModel.availableProviderIDs.isEmpty {
                    Text("No providers configured. Set OPENAI_API_KEY or enable Foundation Models in the runtime environment.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                } else {
                    Picker("Provider", selection: $model.selectedProviderID) {
                        ForEach(appModel.availableProviderIDs, id: \.self) { id in
                            Text(id.capitalized).tag(id)
                        }
                    }
                }
            }

            Section("Safety") {
                LabeledContent("Known Presets") {
                    Text(appModel.safetyGate.knownPresets.sorted().joined(separator: ", "))
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                LabeledContent("Workspace Bounds") {
                    Text(
                        "x [\(appModel.safetyGate.workspaceBounds.xMin), \(appModel.safetyGate.workspaceBounds.xMax)]\n"
                            + "y [\(appModel.safetyGate.workspaceBounds.yMin), \(appModel.safetyGate.workspaceBounds.yMax)]\n"
                            + "z [\(appModel.safetyGate.workspaceBounds.zMin), \(appModel.safetyGate.workspaceBounds.zMax)]"
                    )
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        }
        .formStyle(.grouped)
    }

    private var remoteBootStateColor: Color {
        switch appModel.remoteBootState {
        case .ready:
            return .green
        case .connectingOperator, .connectingNode, .selectingAgent, .syncingState, .readingConfig:
            return .orange
        case .faulted:
            return .red
        }
    }

    private func positionSummary(_ pose: RobotToolPose?) -> String {
        guard let position = pose?.position else {
            return "Unavailable"
        }
        return String(format: "(%.3f, %.3f, %.3f)", position.x, position.y, position.z)
    }
}
