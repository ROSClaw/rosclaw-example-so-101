import SwiftUI
import RosClawBridge

/// Main chat-style control panel — compact floating window.
struct CopilotPanelView: View {
    @Environment(AppCoordinator.self) private var coordinator
    @Environment(RobotConnectionModel.self) private var connection
    @Environment(RobotStateModel.self) private var state
    @Environment(CommandFlowModel.self) private var commandFlow
    @Environment(SpatialSessionModel.self) private var spatial
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace
    @State private var showSettings = false
    @State private var showDiagnostics = false

    var body: some View {
        Group {
            if spatial.immersiveSpaceState == .open {
                compactImmersivePanel
            } else {
                fullPanel
            }
        }
        .sheet(isPresented: $showSettings) {
            SettingsSheet()
        }
        .sheet(isPresented: $showDiagnostics) {
            DiagnosticsSheet()
        }
    }

    private var fullPanel: some View {
        VStack(spacing: 0) {
            topBar
            Divider()

            ChatStreamView()
                .frame(maxWidth: .infinity, maxHeight: .infinity)

            Divider()

            QuickActionStrip()
                .padding(.vertical, 8)

            Divider()

            CommandInputBar()
        }
    }

    private var compactImmersivePanel: some View {
        VStack(spacing: 0) {
            topBar
            Divider()

            VStack(alignment: .leading, spacing: 14) {
                Label("Immersive Control Active", systemImage: "view.3d")
                    .font(.headline.weight(.bold))

                VStack(alignment: .leading, spacing: 6) {
                    Text(compactStageTitle)
                        .font(.subheadline.weight(.semibold))
                    Text(compactStageMessage)
                        .font(.callout)
                        .foregroundStyle(.secondary)
                        .lineLimit(3)
                }

                HStack(spacing: 10) {
                    Button("E-STOP") {
                        Task { await commandFlow.triggerEmergencyStop() }
                    }
                    .buttonStyle(.borderedProminent)
                    .tint(.red)

                    Button("Exit Spatial") {
                        Task { await dismissImmersiveSpace() }
                    }
                    .buttonStyle(.bordered)
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(16)
        }
    }

    private var topBar: some View {
        HStack(spacing: 12) {
            // Connection indicator
            Circle()
                .fill(connectionColor)
                .frame(width: 10, height: 10)

            Text("Spatial Copilot")
                .font(.headline.weight(.bold))

            Spacer()

            // Phase badge
            Text(commandFlow.appPhase.rawValue)
                .font(.caption.weight(.bold))
                .padding(.horizontal, 10)
                .padding(.vertical, 4)
                .background(phaseColor.opacity(0.2), in: Capsule())
                .foregroundStyle(phaseColor)

            // Diagnostics
            Button { showDiagnostics = true } label: {
                Image(systemName: "chart.bar")
                    .font(.body.weight(.semibold))
            }
            .buttonStyle(.plain)
            .hoverEffect()

            // Settings
            Button { showSettings = true } label: {
                Image(systemName: "gear")
                    .font(.body.weight(.semibold))
            }
            .buttonStyle(.plain)
            .hoverEffect()
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 12)
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

    private var compactStageTitle: String {
        switch spatial.onboardingStage {
        case .needsStylus:
            return "Waiting for stylus"
        case .needsRobotState:
            return "Waiting for robot pose"
        case .placingBase:
            return "Place the robot base"
        case .placingGripper:
            return "Place the gripper point"
        case .previewing:
            return "Previewing target"
        case .confirming:
            return "Confirming spatial move"
        }
    }

    private var compactStageMessage: String {
        switch spatial.onboardingStage {
        case .needsStylus:
            return "Pair the stylus before immersive calibration can continue."
        case .needsRobotState:
            return state.toolPoseStatusSummary
        case .placingBase:
            return "Use the stylus to place the base on a recognized surface. The point turns green when placement is valid and red when it is not."
        case .placingGripper:
            return "Point the stylus at the real gripper location in free space. The virtual gripper follows the stylus while the point stays green only inside the safe zone."
        case .previewing:
            return "Move the stylus to preview the gripper target. Green means the point is safe to control or confirm; red means bring it back into the reachable zone before holding or confirming."
        case .confirming:
            return "Use the in-scene confirmation card or the stylus secondary button to cancel."
        }
    }
}
