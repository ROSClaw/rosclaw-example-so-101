import SwiftUI
import OpenClawCore

/// Diagnostics presented as a sheet.
struct DiagnosticsSheet: View {
    @Environment(CommandFlowModel.self) private var commandFlow
    @Environment(\.dismiss) private var dismiss
    @State private var events: [RuntimeDiagnosticEvent] = []

    var body: some View {
        NavigationStack {
            VStack(alignment: .leading, spacing: 16) {
                if events.isEmpty {
                    Text("No diagnostic events yet.")
                        .font(.title3.weight(.medium))
                        .foregroundStyle(.secondary)
                        .frame(maxWidth: .infinity, alignment: .center)
                        .padding(.vertical, 32)
                } else {
                    ScrollView {
                        LazyVStack(alignment: .leading, spacing: 10) {
                            ForEach(events.reversed(), id: \.occurredAt) { event in
                                HStack(alignment: .top, spacing: 12) {
                                    Circle()
                                        .fill(subsystemColor(event.subsystem))
                                        .frame(width: 10, height: 10)
                                        .padding(.top, 6)
                                    VStack(alignment: .leading, spacing: 2) {
                                        Text("\(event.subsystem).\(event.name)")
                                            .font(.body.monospaced().weight(.semibold))
                                        if let runID = event.runID {
                                            Text("run: \(runID.prefix(8))")
                                                .font(.caption.monospaced())
                                                .foregroundStyle(.secondary)
                                        }
                                    }
                                    Spacer()
                                    Text(event.occurredAt, style: .time)
                                        .font(.caption.monospacedDigit().weight(.medium))
                                        .foregroundStyle(.tertiary)
                                }
                                .padding(12)
                                .glassBackgroundEffect(in: RoundedRectangle(cornerRadius: 12, style: .continuous))
                            }
                        }
                        .padding(.horizontal, 16)
                    }
                }
            }
            .navigationTitle("Diagnostics")
            .toolbar { ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } } }
            .task { await refreshLoop() }
        }
    }

    private func refreshLoop() async {
        while !Task.isCancelled {
            events = await commandFlow.diagnosticsPipeline.recentEvents(limit: 50)
            try? await Task.sleep(nanoseconds: 1_000_000_000)
        }
    }

    private func subsystemColor(_ subsystem: String) -> Color {
        switch subsystem {
        case "runtime": return .blue
        case "channel": return .green
        case "security": return .orange
        default: return .secondary
        }
    }
}
