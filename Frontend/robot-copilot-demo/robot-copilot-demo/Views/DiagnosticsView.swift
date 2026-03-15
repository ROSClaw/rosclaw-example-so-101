import SwiftUI
import OpenClawCore

struct DiagnosticsView: View {
    @Environment(AppModel.self) private var appModel
    @State private var events: [RuntimeDiagnosticEvent] = []
    @State private var refreshTimer: Timer?

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Label("Diagnostics", systemImage: "chart.bar")
                    .font(.headline)
                Spacer()
                Button("Clear") {
                    Task { await appModel.diagnosticsPipeline.reset() }
                }
                .font(.caption)
                .buttonStyle(.bordered)
            }

            if events.isEmpty {
                Text("No events yet")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .frame(maxWidth: .infinity, alignment: .center)
                    .padding(.vertical, 8)
            } else {
                ScrollView {
                    LazyVStack(alignment: .leading, spacing: 4) {
                        ForEach(events.reversed(), id: \.occurredAt) { event in
                            DiagnosticEventRow(event: event)
                        }
                    }
                }
                .frame(maxHeight: 200)
            }
        }
        .padding()
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
        .task {
            await refreshEvents()
        }
    }

    private func refreshEvents() async {
        while !Task.isCancelled {
            events = await appModel.diagnosticsPipeline.recentEvents(limit: 50)
            try? await Task.sleep(nanoseconds: 1_000_000_000) // 1s refresh
        }
    }
}

private struct DiagnosticEventRow: View {
    let event: RuntimeDiagnosticEvent

    var body: some View {
        HStack(alignment: .top, spacing: 6) {
            Circle()
                .fill(subsystemColor)
                .frame(width: 6, height: 6)
                .padding(.top, 4)

            VStack(alignment: .leading, spacing: 1) {
                Text("\(event.subsystem).\(event.name)")
                    .font(.caption.monospaced())
                    .foregroundStyle(.primary)
                if let runID = event.runID {
                    Text("run: \(runID.prefix(8))")
                        .font(.caption2)
                        .foregroundStyle(.secondary)
                }
            }

            Spacer()

            Text(event.occurredAt, style: .time)
                .font(.caption2.monospacedDigit())
                .foregroundStyle(.tertiary)
        }
    }

    private var subsystemColor: Color {
        switch event.subsystem {
        case "runtime": return .blue
        case "channel": return .green
        case "security": return .orange
        default: return .secondary
        }
    }
}
