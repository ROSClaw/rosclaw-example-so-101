import SwiftUI
import RosClawBridge

/// Compact floating badge attached near the robot's end-effector in the immersive space.
struct RobotStatusBadge: View {
    @Environment(RobotStateModel.self) private var state
    @Environment(CommandFlowModel.self) private var commandFlow

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 8) {
                Circle()
                    .fill(connectionColor)
                    .frame(width: 10, height: 10)
                Text(state.robotState.connectionState.rawValue.capitalized)
                    .font(.callout.weight(.bold))
            }

            Text("Gripper: \(state.robotState.gripperState.rawValue.capitalized)")
                .font(.callout)
                .foregroundStyle(.secondary)

            Text("Pose: \(poseStatusTitle)")
                .font(.callout)
                .foregroundStyle(state.robotState.toolPose?.position != nil ? .green : .orange)

            if state.robotState.toolPose?.position == nil {
                Text(state.toolPoseStatusSummary)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .lineLimit(3)
            }

            if let fault = state.robotState.fault {
                Label(fault, systemImage: "exclamationmark.triangle.fill")
                    .font(.callout.weight(.medium))
                    .foregroundStyle(.red)
                    .lineLimit(2)
            }

            Text(commandFlow.appPhase.rawValue)
                .font(.callout.weight(.bold))
                .padding(.horizontal, 10)
                .padding(.vertical, 4)
                .background(phaseColor.opacity(0.2), in: Capsule())
                .foregroundStyle(phaseColor)
        }
        .padding(16)
        .glassBackgroundEffect()
    }

    private var connectionColor: Color {
        switch state.robotState.connectionState {
        case .connected: return .green
        case .connecting: return .orange
        case .disconnected: return .red
        }
    }

    private var phaseColor: Color {
        switch commandFlow.appPhase {
        case .idle: return .secondary
        case .previewing: return .blue
        case .awaitingClarification, .awaitingConfirmation: return .orange
        case .executing: return .green
        case .faulted: return .red
        }
    }

    private var poseStatusTitle: String {
        if state.robotState.toolPose?.position != nil {
            return "Ready"
        }
        guard let stage = state.toolPoseStatus?.stage else {
            return "Unavailable"
        }
        return stage.replacingOccurrences(of: "_", with: " ").capitalized
    }
}
