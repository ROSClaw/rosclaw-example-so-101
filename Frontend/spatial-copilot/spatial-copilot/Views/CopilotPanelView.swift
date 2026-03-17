import SwiftUI
import RosClawBridge

/// Main chat-style control panel — compact floating window.
struct CopilotPanelView: View {
    @Environment(AppCoordinator.self) private var coordinator
    @Environment(RobotConnectionModel.self) private var connection
    @Environment(RobotStateModel.self) private var state
    @Environment(CommandFlowModel.self) private var commandFlow
    @Environment(SpatialSessionModel.self) private var spatial
    @State private var showSettings = false
    @State private var showDiagnostics = false

    var body: some View {
        VStack(spacing: 0) {
            // Top bar
            topBar
            Divider()

            // Chat stream
            ChatStreamView()
                .frame(maxWidth: .infinity, maxHeight: .infinity)

            Divider()

            // Quick actions
            QuickActionStrip()
                .padding(.vertical, 8)

            Divider()

            // Input bar
            CommandInputBar()
        }
        .sheet(isPresented: $showSettings) {
            SettingsSheet()
        }
        .sheet(isPresented: $showDiagnostics) {
            DiagnosticsSheet()
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
}
