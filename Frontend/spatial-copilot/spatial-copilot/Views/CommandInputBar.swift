import SwiftUI
import RosClawBridge

/// Text field + mic button + send button — anchored at the bottom of the copilot panel.
struct CommandInputBar: View {
    @Environment(CommandFlowModel.self) private var commandFlow
    @State private var speechManager = SpeechInputManager()
    @State private var textInput: String = ""
    @FocusState private var textFieldFocused: Bool

    var body: some View {
        VStack(spacing: 12) {
            if !speechManager.partialTranscript.isEmpty {
                Text(speechManager.partialTranscript)
                    .font(.callout.weight(.medium))
                    .foregroundStyle(.secondary)
                    .lineLimit(2)
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .padding(.horizontal, 12)
            }

            HStack(spacing: 12) {
                // Mic button
                Button {
                    Task { await toggleSpeech() }
                } label: {
                    Image(systemName: speechManager.state == .listening ? "waveform.circle.fill" : "mic.circle.fill")
                        .font(.system(size: 28))
                        .foregroundStyle(speechManager.state == .listening ? .red : .primary)
                }
                .buttonStyle(.plain)
                .frame(width: 44, height: 44)
                .hoverEffect()

                // Text field
                TextField("Type a command...", text: $textInput)
                    .textFieldStyle(.roundedBorder)
                    .font(.body.weight(.medium))
                    .focused($textFieldFocused)
                    .disabled(commandFlow.appPhase == .executing)
                    .onSubmit { submitText() }

                // Send button
                Button { submitText() } label: {
                    Image(systemName: "arrow.up.circle.fill")
                        .font(.system(size: 28))
                        .foregroundStyle(.blue)
                }
                .buttonStyle(.plain)
                .frame(width: 44, height: 44)
                .disabled(textInput.trimmingCharacters(in: .whitespaces).isEmpty || commandFlow.appPhase == .executing)
                .hoverEffect()
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 12)
    }

    private func toggleSpeech() async {
        if speechManager.state == .listening {
            let transcript = speechManager.stopListening()
            if !transcript.isEmpty { await commandFlow.handleInput(transcript) }
        } else {
            await speechManager.startListening()
        }
    }

    private func submitText() {
        let text = textInput.trimmingCharacters(in: .whitespaces)
        guard !text.isEmpty else { return }
        textInput = ""
        Task { await commandFlow.handleInput(text) }
    }
}
