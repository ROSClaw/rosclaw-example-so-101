import SwiftUI
import RosClawBridge

struct CommandInputView: View {
    @Environment(AppModel.self) private var appModel
    @State private var speechManager = SpeechInputManager()
    @State private var textInput: String = ""
    @FocusState private var textFieldFocused: Bool

    var body: some View {
        VStack(spacing: 12) {
            // Push-to-talk button
            Button {
                Task { await toggleSpeech() }
            } label: {
                HStack(spacing: 8) {
                    Image(systemName: speechManager.state == .listening ? "mic.fill" : "mic")
                        .foregroundStyle(speechManager.state == .listening ? .red : .primary)
                    Text(speechManager.state == .listening ? "Listening..." : "Hold to Speak")
                        .font(.headline)
                }
                .frame(maxWidth: .infinity)
                .padding(.vertical, 12)
            }
            .buttonStyle(.borderedProminent)
            .tint(speechManager.state == .listening ? .red : .blue)
            .disabled(appModel.appPhase == .executing)

            // Partial transcript preview
            if !speechManager.partialTranscript.isEmpty {
                Text(speechManager.partialTranscript)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .lineLimit(2)
            }

            // Text input fallback
            HStack {
                TextField("Type a command...", text: $textInput)
                    .textFieldStyle(.roundedBorder)
                    .focused($textFieldFocused)
                    .disabled(appModel.appPhase == .executing)
                    .onSubmit { submitText() }

                Button("Send") { submitText() }
                    .disabled(textInput.trimmingCharacters(in: .whitespaces).isEmpty || appModel.appPhase == .executing)
            }
        }
        .padding()
    }

    private func toggleSpeech() async {
        if speechManager.state == .listening {
            let transcript = speechManager.stopListening()
            if !transcript.isEmpty {
                await appModel.handleInput(transcript)
            }
        } else {
            await speechManager.startListening()
        }
    }

    private func submitText() {
        let text = textInput.trimmingCharacters(in: .whitespaces)
        guard !text.isEmpty else { return }
        textInput = ""
        Task { await appModel.handleInput(text) }
    }
}
