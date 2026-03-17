import SwiftUI
import RosClawBridge

/// Inline command preview card with confirm/cancel — shown in the chat stream.
struct CommandPreviewCard: View {
    @Environment(CommandFlowModel.self) private var commandFlow

    var body: some View {
        if let preview = commandFlow.pendingPreview {
            VStack(alignment: .leading, spacing: 12) {
                Label("Command Preview", systemImage: "eye.circle.fill")
                    .font(.headline.weight(.bold))

                HStack {
                    Text("Command").font(.subheadline.weight(.medium)).foregroundStyle(.secondary)
                    Spacer()
                    Text(preview.parsedCommandName).font(.subheadline.weight(.bold))
                }

                if !preview.targets.isEmpty {
                    HStack(alignment: .top) {
                        Text("Targets").font(.subheadline.weight(.medium)).foregroundStyle(.secondary)
                        Spacer()
                        VStack(alignment: .trailing, spacing: 2) {
                            ForEach(preview.targets, id: \.self) { t in
                                Text(t).font(.caption.weight(.semibold))
                            }
                        }
                    }
                }

                HStack(spacing: 8) {
                    Image(systemName: preview.validationResult.isAllowed ? "checkmark.circle.fill" : "xmark.circle.fill")
                    Text(preview.validationResult.isAllowed ? "Safe to execute" : (preview.validationResult.blockReason ?? "Blocked"))
                        .font(.subheadline.weight(.semibold))
                }
                .foregroundStyle(preview.validationResult.isAllowed ? .green : .red)

                HStack(spacing: 12) {
                    Button("Cancel") { commandFlow.cancelPending() }
                        .buttonStyle(.bordered).tint(.secondary).hoverEffect()
                    Button("Execute") { Task { await commandFlow.confirmExecution() } }
                        .buttonStyle(.borderedProminent).tint(.blue)
                        .disabled(!preview.canExecute).hoverEffect()
                }
                .frame(maxWidth: .infinity)
            }
            .padding(16)
            .glassBackgroundEffect(in: RoundedRectangle(cornerRadius: 20, style: .continuous))
            .padding3D(.front, 16)
            .overlay(
                RoundedRectangle(cornerRadius: 20, style: .continuous)
                    .stroke(preview.validationResult.isAllowed ? Color.blue.opacity(0.5) : Color.red.opacity(0.5), lineWidth: 1.5)
            )
        }
    }
}
