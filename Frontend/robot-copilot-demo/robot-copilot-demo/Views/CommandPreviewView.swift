import SwiftUI
import RosClawBridge

struct CommandPreviewView: View {
    @Environment(AppModel.self) private var appModel

    var body: some View {
        if let preview = appModel.pendingPreview {
            VStack(alignment: .leading, spacing: 12) {
                Label("Command Preview", systemImage: "eye")
                    .font(.headline)

                // Command name
                HStack {
                    Text("Command")
                        .font(.caption).foregroundStyle(.secondary)
                    Spacer()
                    Text(preview.parsedCommandName)
                        .font(.caption.weight(.semibold))
                }

                // Targets
                if !preview.targets.isEmpty {
                    HStack(alignment: .top) {
                        Text("Targets")
                            .font(.caption).foregroundStyle(.secondary)
                        Spacer()
                        VStack(alignment: .trailing) {
                            ForEach(preview.targets, id: \.self) { target in
                                Text(target).font(.caption.weight(.medium))
                            }
                        }
                    }
                }

                // Subsystems
                HStack(alignment: .top) {
                    Text("Subsystems")
                        .font(.caption).foregroundStyle(.secondary)
                    Spacer()
                    Text(preview.subsystemsTouched.joined(separator: ", "))
                        .font(.caption)
                        .multilineTextAlignment(.trailing)
                }

                // Validation result
                HStack {
                    Image(systemName: preview.validationResult.isAllowed ? "checkmark.circle.fill" : "xmark.circle.fill")
                        .foregroundStyle(preview.validationResult.isAllowed ? .green : .red)
                    Text(preview.validationResult.isAllowed ? "Safe to execute" : (preview.validationResult.blockReason ?? "Blocked"))
                        .font(.caption)
                        .foregroundStyle(preview.validationResult.isAllowed ? .green : .red)
                }

                Divider()

                // Action buttons
                HStack(spacing: 12) {
                    Button("Cancel") {
                        appModel.cancelPending()
                    }
                    .buttonStyle(.bordered)
                    .tint(.secondary)

                    Button("Execute") {
                        Task { await appModel.confirmExecution() }
                    }
                    .buttonStyle(.borderedProminent)
                    .tint(.blue)
                    .disabled(!preview.canExecute)
                }
                .frame(maxWidth: .infinity)
            }
            .padding()
            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 12))
            .overlay(
                RoundedRectangle(cornerRadius: 12)
                    .stroke(preview.validationResult.isAllowed ? Color.blue.opacity(0.4) : Color.red.opacity(0.4), lineWidth: 1)
            )
        }
    }
}
