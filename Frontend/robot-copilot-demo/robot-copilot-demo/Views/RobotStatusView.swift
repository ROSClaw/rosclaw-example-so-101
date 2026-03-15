import SwiftUI
import RosClawBridge

struct RobotStatusView: View {
    @Environment(AppModel.self) private var appModel

    private struct JointRow: Identifiable {
        let id: String
        let value: Double
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            Label("Robot Status", systemImage: "cpu")
                .font(.headline)

            Grid(alignment: .leading, horizontalSpacing: 16, verticalSpacing: 6) {
                statusRow("Connection", value: appModel.robotState.connectionState.rawValue,
                          color: connectionColor)
                statusRow("Enabled", value: appModel.robotState.enabled ? "Yes" : "No",
                          color: appModel.robotState.enabled ? .green : .orange)
                statusRow("Mode", value: appModel.robotState.mode.rawValue.capitalized,
                          color: modeColor)
                statusRow("Gripper", value: appModel.robotState.gripperState.rawValue.capitalized,
                          color: .primary)
                statusRow("Action", value: appModel.robotState.actionStatus.rawValue.capitalized,
                          color: actionColor)
                if appModel.robotState.estopLatched {
                    statusRow("E-Stop", value: "Latched", color: .red)
                }
                if let fault = appModel.robotState.fault {
                    statusRow("Fault", value: fault, color: .red)
                }
                if let currentNamedPose = appModel.currentNamedPose {
                    statusRow("Named Pose", value: currentNamedPose, color: .secondary)
                }
                if let resolution = appModel.lastResolvedCartesianGoal {
                    statusRow("Requested", value: positionSummary(resolution.requestedPose), color: .secondary)
                    statusRow("Resolved", value: positionSummary(resolution.resolvedPose), color: .secondary)
                }
            }

            // Joint summary (collapsed)
            if !appModel.robotState.jointSummary.isEmpty {
                DisclosureGroup("Joints (\(appModel.robotState.jointSummary.count))") {
                    ForEach(orderedJointRows) { joint in
                        HStack {
                            Text(joint.id).font(.caption).foregroundStyle(.secondary)
                            Spacer()
                            Text(String(format: "%.3f rad", joint.value)).font(.caption.monospacedDigit()
                            )
                        }
                    }
                }
                .font(.caption)
            }
        }
        .padding()
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
    }

    private func statusRow(_ label: String, value: String, color: Color) -> some View {
        GridRow {
            Text(label)
                .font(.caption)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.caption.weight(.medium))
                .foregroundStyle(color)
        }
    }

    private var connectionColor: Color {
        switch appModel.robotState.connectionState {
        case .connected: return .green
        case .connecting: return .orange
        case .disconnected: return .red
        }
    }

    private var modeColor: Color {
        switch appModel.robotState.mode {
        case .faulted: return .red
        case .executing: return .blue
        case .idle: return .green
        default: return .secondary
        }
    }

    private var actionColor: Color {
        switch appModel.robotState.actionStatus {
        case .executing: return .blue
        case .succeeded: return .green
        case .failed: return .red
        case .cancelled: return .orange
        case .idle: return .secondary
        }
    }

    private var orderedJointRows: [JointRow] {
        if !appModel.robotState.jointOrder.isEmpty {
            return appModel.robotState.jointOrder.compactMap { name in
                guard let value = appModel.robotState.jointPositions[name] else { return nil }
                return JointRow(id: name, value: value)
            }
        }
        return appModel.robotState.jointSummary.sorted(by: { $0.key < $1.key })
            .map { JointRow(id: $0.key, value: $0.value) }
    }

    private func positionSummary(_ pose: RobotToolPose?) -> String {
        guard let position = pose?.position else {
            return "Unavailable"
        }
        return String(format: "(%.3f, %.3f, %.3f)", position.x, position.y, position.z)
    }
}
