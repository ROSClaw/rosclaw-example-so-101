import Foundation
import Speech
import AVFoundation

/// Push-to-talk speech recognition manager.
/// Uses SFSpeechRecognizer + AVAudioEngine for on-device transcription.
@Observable
@MainActor
final class SpeechInputManager {

    enum State: Equatable {
        case idle
        case listening
        case processing
        case unavailable(String)
    }

    var state: State = .idle
    var partialTranscript: String = ""

    private var recognizer: SFSpeechRecognizer?
    private var audioEngine: AVAudioEngine?
    private var recognitionRequest: SFSpeechAudioBufferRecognitionRequest?
    private var recognitionTask: SFSpeechRecognitionTask?

    init() {
        recognizer = SFSpeechRecognizer(locale: Locale.current)
    }

    // MARK: - Authorization

    func requestAuthorization() async -> Bool {
        let status = await withCheckedContinuation { continuation in
            SFSpeechRecognizer.requestAuthorization { status in
                continuation.resume(returning: status)
            }
        }
        switch status {
        case .authorized: return true
        default:
            state = .unavailable("Speech recognition not authorized")
            return false
        }
    }

    // MARK: - Push-to-talk

    func startListening() async {
        guard await requestAuthorization() else { return }
        guard let recognizer, recognizer.isAvailable else {
            state = .unavailable("Speech recognizer unavailable")
            return
        }

        stopListening()

        let engine = AVAudioEngine()
        audioEngine = engine
        let request = SFSpeechAudioBufferRecognitionRequest()
        request.shouldReportPartialResults = true
        recognitionRequest = request

        let inputNode = engine.inputNode
        let format = inputNode.outputFormat(forBus: 0)
        inputNode.installTap(onBus: 0, bufferSize: 1024, format: format) { [weak self] buffer, _ in
            self?.recognitionRequest?.append(buffer)
        }

        engine.prepare()
        do {
            try engine.start()
        } catch {
            state = .unavailable("Audio engine failed: \(error.localizedDescription)")
            return
        }

        state = .listening
        partialTranscript = ""

        recognitionTask = recognizer.recognitionTask(with: request) { [weak self] result, error in
            guard let self else { return }
            if let result {
                Task { @MainActor in
                    self.partialTranscript = result.bestTranscription.formattedString
                }
            }
            if error != nil || result?.isFinal == true {
                Task { @MainActor in
                    self.state = .idle
                }
            }
        }
    }

    /// Stops listening and returns the final transcript.
    @discardableResult
    func stopListening() -> String {
        let transcript = partialTranscript
        recognitionRequest?.endAudio()
        recognitionTask?.cancel()
        audioEngine?.stop()
        audioEngine?.inputNode.removeTap(onBus: 0)
        recognitionRequest = nil
        recognitionTask = nil
        audioEngine = nil
        state = .idle
        partialTranscript = ""
        return transcript
    }
}
