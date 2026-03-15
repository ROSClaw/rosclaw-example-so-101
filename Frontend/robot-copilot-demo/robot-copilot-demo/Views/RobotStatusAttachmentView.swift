import SwiftUI
import RealityKit
import RosClawBridge

struct RobotStatusAttachmentView: View {
    @Environment(AppModel.self) private var appModel

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack(spacing: 6) {
                Circle()
                    .fill(connectionColor)
                    .frame(width: 8, height: 8)
                Text(appModel.robotState.connectionState.rawValue.capitalized)
                    .font(.caption.weight(.semibold))
            }

            Text("Gripper: \(appModel.robotState.gripperState.rawValue.capitalized)")
                .font(.caption2)
                .foregroundStyle(.secondary)

            if let fault = appModel.robotState.fault {
                Label(fault, systemImage: "exclamationmark.triangle.fill")
                    .font(.caption2)
                    .foregroundStyle(.red)
            }

            // Phase badge
            Text(appModel.appPhase.rawValue)
                .font(.caption2.weight(.medium))
                .padding(.horizontal, 6)
                .padding(.vertical, 2)
                .background(phaseColor.opacity(0.2), in: Capsule())
                .foregroundStyle(phaseColor)
        }
        .padding(8)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 8))
    }

    private var connectionColor: Color {
        switch appModel.robotState.connectionState {
        case .connected: return .green
        case .connecting: return .orange
        case .disconnected: return .red
        }
    }

    private var phaseColor: Color {
        switch appModel.appPhase {
        case .idle: return .secondary
        case .previewing: return .blue
        case .awaitingConfirmation: return .orange
        case .executing: return .green
        case .faulted: return .red
        }
    }
}
