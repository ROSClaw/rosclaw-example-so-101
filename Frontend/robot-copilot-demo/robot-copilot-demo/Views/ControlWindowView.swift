import SwiftUI
import RosClawBridge

struct ControlWindowView: View {
    @Environment(AppModel.self) private var appModel
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace
    @State private var selectedTab: Tab = .command

    enum Tab: String, CaseIterable {
        case command = "Command"
        case status = "Status"
        case diagnostics = "Diagnostics"
        case settings = "Settings"

        var icon: String {
            switch self {
            case .command: return "mic.circle"
            case .status: return "cpu"
            case .diagnostics: return "chart.bar"
            case .settings: return "gear"
            }
        }
    }

    var body: some View {
        NavigationSplitView {
            // Sidebar
            List(Tab.allCases, id: \.self, selection: Binding(
                get: { selectedTab },
                set: { selectedTab = $0 ?? .command }
            )) { tab in
                Label(tab.rawValue, systemImage: tab.icon)
            }
            .navigationTitle("Spatial Copilot")
        } detail: {
            Group {
                switch selectedTab {
                case .command:
                    commandPanel
                case .status:
                    ScrollView { RobotStatusView().padding() }
                case .diagnostics:
                    ScrollView { DiagnosticsView().padding() }
                case .settings:
                    SettingsView()
                }
            }
        }
        .ornament(attachmentAnchor: .scene(.bottom)) {
            bottomOrnament
        }
    }

    // MARK: - Command panel

    private var commandPanel: some View {
        ScrollView {
            VStack(spacing: 16) {
                // Phase badge
                phaseBadge

                // Command input
                CommandInputView()

                // Preview (shown when pending)
                if appModel.pendingPreview != nil {
                    CommandPreviewView()
                        .padding(.horizontal)
                }

                // Transcript
                transcriptView
            }
            .padding()
        }
    }

    private var phaseBadge: some View {
        HStack {
            Circle()
                .fill(phaseColor)
                .frame(width: 8, height: 8)
            Text(appModel.appPhase.rawValue)
                .font(.caption.weight(.semibold))
                .foregroundStyle(phaseColor)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 4)
        .background(phaseColor.opacity(0.1), in: Capsule())
    }

    private var transcriptView: some View {
        VStack(alignment: .leading, spacing: 4) {
            Label("Transcript", systemImage: "text.bubble")
                .font(.headline)
                .padding(.horizontal)

            if appModel.transcript.isEmpty {
                Text("No commands yet. Speak or type a command above.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .padding()
            } else {
                ForEach(appModel.transcript.suffix(20).reversed()) { entry in
                    TranscriptEntryRow(entry: entry)
                }
            }
        }
    }

    // MARK: - Bottom ornament (§8.3)

    private var bottomOrnament: some View {
        HStack(spacing: 16) {
            // E-stop
            Button {
                Task { await appModel.triggerEmergencyStop() }
            } label: {
                Label("E-STOP", systemImage: "stop.circle.fill")
                    .foregroundStyle(.white)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 8)
            }
            .background(.red, in: RoundedRectangle(cornerRadius: 8))
            .buttonStyle(.plain)

            Divider().frame(height: 24)

            // Connection status
            HStack(spacing: 4) {
                Circle()
                    .fill(connectionColor)
                    .frame(width: 8, height: 8)
                Text(appModel.robotState.connectionState.rawValue.capitalized)
                    .font(.caption)
            }

            Divider().frame(height: 24)

            // Spatial view toggle
            Button {
                Task {
                    if appModel.immersiveSpaceState == .open {
                        await dismissImmersiveSpace()
                    } else {
                        await openImmersiveSpace(id: appModel.immersiveSpaceID)
                    }
                }
            } label: {
                Label(
                    appModel.immersiveSpaceState == .open ? "Exit Spatial" : "Spatial View",
                    systemImage: appModel.immersiveSpaceState == .open ? "arrow.down.right.and.arrow.up.left" : "view.3d"
                )
                .font(.caption)
            }
            .buttonStyle(.bordered)
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 8)
        .glassBackgroundEffect()
    }

    // MARK: - Helpers

    private var phaseColor: Color {
        switch appModel.appPhase {
        case .idle: return .secondary
        case .previewing: return .blue
        case .awaitingConfirmation: return .orange
        case .executing: return .green
        case .faulted: return .red
        }
    }

    private var connectionColor: Color {
        switch appModel.robotState.connectionState {
        case .connected: return .green
        case .connecting: return .orange
        case .disconnected: return .red
        }
    }
}

// MARK: - Transcript row

private struct TranscriptEntryRow: View {
    let entry: TranscriptEntry

    var body: some View {
        HStack(alignment: .top, spacing: 8) {
            Image(systemName: icon)
                .foregroundStyle(color)
                .frame(width: 16)
            VStack(alignment: .leading, spacing: 2) {
                Text(entry.text)
                    .font(.caption)
                Text(entry.timestamp, style: .time)
                    .font(.caption2)
                    .foregroundStyle(.tertiary)
            }
        }
        .padding(.horizontal)
        .padding(.vertical, 2)
    }

    private var icon: String {
        switch entry.kind {
        case .userInput: return "person.fill"
        case .parsedCommand: return "arrow.right.circle"
        case .feedback: return "checkmark.circle"
        case .error: return "exclamationmark.circle"
        case .system: return "info.circle"
        }
    }

    private var color: Color {
        switch entry.kind {
        case .userInput: return .blue
        case .parsedCommand: return .purple
        case .feedback: return .green
        case .error: return .red
        case .system: return .secondary
        }
    }
}

#Preview(windowStyle: .automatic) {
    ControlWindowView()
        .environment(AppModel())
}
