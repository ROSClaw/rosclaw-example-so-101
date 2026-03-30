import Foundation
import OpenClawAgents
import OpenClawCore
import OpenClawKit
import OpenClawModels
import RosClawBridge

enum AppPhase: String, Sendable {
    case idle = "Idle"
    case previewing = "Previewing"
    case awaitingClarification = "Awaiting Clarification"
    case awaitingConfirmation = "Awaiting Confirmation"
    case executing = "Executing"
    case faulted = "Faulted"
}

struct TranscriptEntry: Identifiable, Sendable {
    enum Kind: Sendable {
        case userInput, parsedCommand, feedback, error, system, perception
    }
    let id = UUID()
    let kind: Kind
    let text: String
    let perceptionSnapshot: PerceptionSnapshot?
    let timestamp: Date = Date()

    init(kind: Kind, text: String, perceptionSnapshot: PerceptionSnapshot? = nil) {
        self.kind = kind
        self.text = text
        self.perceptionSnapshot = perceptionSnapshot
    }
}

struct PendingCartesianClarificationDraft: Sendable, Equatable {
    let x: Double
    let y: Double
    let z: Double
    let referenceFrame: RobotCartesianReferenceFrame
    let source: RobotCartesianSource
}

struct PendingCartesianClarification: Sendable, Equatable {
    let question: String
    let draft: PendingCartesianClarificationDraft
    let remainingAttempts: Int
    init(question: String, draft: PendingCartesianClarificationDraft, remainingAttempts: Int = 2) {
        self.question = question
        self.draft = draft
        self.remainingAttempts = remainingAttempts
    }
}

enum ParsedUserInput: Equatable {
    case command(RobotCommand)
    case clarification(PendingCartesianClarification)
}

/// Command parsing, preview, execution, and transcript management.
@MainActor
@Observable
final class CommandFlowModel {

    var appPhase: AppPhase = .idle
    var pendingCommand: RobotCommand?
    var pendingPreview: CommandPreview?
    var pendingClarification: PendingCartesianClarification?
    var transcript: [TranscriptEntry] = []

    let agentRuntime: EmbeddedAgentRuntime
    let diagnosticsPipeline: RuntimeDiagnosticsPipeline
    var availableProviderIDs: [String] = []
    var selectedProviderID: String = ""

    weak var connectionModel: RobotConnectionModel?
    weak var stateModel: RobotStateModel?
    weak var spatialModel: SpatialSessionModel?

    private var hasSetupModelProviders = false

    init() {
        let sdk = OpenClawSDK.shared
        self.diagnosticsPipeline = sdk.makeDiagnosticsPipeline(eventLimit: 500)
        self.agentRuntime = EmbeddedAgentRuntime()
    }

    func setupModelProvidersIfNeeded() async {
        guard !hasSetupModelProviders else { return }
        hasSetupModelProviders = true
        await setupModelProviders()
    }

    // MARK: - Input handling

    func handleInput(_ text: String) async {
        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return }
        appendTranscript(.userInput, trimmed)

        let lower = trimmed.lowercased()
        if lower == "cancel" || lower == "never mind" || lower == "nevermind" {
            cancelPending()
            return
        }
        if lower.contains("stop") || lower.contains("estop") || lower.contains("emergency") {
            guard connectionModel?.remoteBootState == .ready else {
                appendTranscript(.error, "The Gateway executor is not ready yet.")
                return
            }
            await triggerEmergencyStop()
            return
        }
        if let clarification = pendingClarification {
            await resolvePendingClarification(with: trimmed, clarification: clarification)
            return
        }
        guard connectionModel?.remoteBootState == .ready else {
            appendTranscript(.error, "The Gateway executor is not ready yet.")
            return
        }
        appPhase = .previewing
        do {
            let parsed = try await parseCommand(trimmed)
            switch parsed {
            case .command(let command):
                preview(command)
            case .clarification(let clarification):
                pendingClarification = clarification
                pendingCommand = nil
                pendingPreview = nil
                appPhase = .awaitingClarification
                appendTranscript(.system, clarification.question)
            }
        } catch {
            appendTranscript(.error, "Parse error: \(error.localizedDescription)")
            appPhase = .idle
        }
    }

    func enterImmersiveMode() {
        guard appPhase != .executing else { return }
        clearPendingInteractiveState(resetPhase: true)
        stateModel?.lastRequestedCartesianTarget = nil
    }

    func executeQuickAction(_ command: RobotCommand) async {
        guard connectionModel?.remoteBootState == .ready else {
            appendTranscript(.error, "The Gateway executor is not ready yet.")
            return
        }
        if case .emergencyStop = command {
            await triggerEmergencyStop()
            return
        }
        guard let stateModel else { return }

        clearPendingInteractiveState(resetPhase: false)

        let safetyResult = stateModel.safetyGate.validate(command, robotState: stateModel.robotState)
        guard safetyResult.isAllowed else {
            appendTranscript(.error, safetyResult.blockReason ?? "Command is not safe to execute.")
            appPhase = .idle
            return
        }

        appendTranscript(.parsedCommand, "Quick action: \(command.displayName)")
        appPhase = .executing
        stateModel.robotState.actionStatus = .executing
        stateModel.robotState.mode = .executing
        appendTranscript(.system, "Executing: \(command.displayName)")

        do {
            try await dispatchCommand(command)
            if case .moveToCartesian = command {
                appendCartesianResolutionTranscriptIfAvailable()
            }
            appendTranscript(.feedback, "Completed: \(command.displayName)")
            appPhase = .idle
        } catch {
            stateModel.robotState.actionStatus = .failed
            stateModel.robotState.mode = .faulted
            stateModel.robotState.fault = error.localizedDescription
            appendTranscript(.error, "Execution failed: \(error.localizedDescription)")
            appPhase = .faulted
        }
    }
// MARK: - Confirm / Cancel / E-Stop

    func confirmExecution() async {
        guard let command = pendingCommand else { return }
        guard let preview = pendingPreview, preview.canExecute else {
            appendTranscript(.error, "Cannot execute: \(pendingPreview?.validationResult.blockReason ?? "unknown reason")")
            cancelPending()
            return
        }
        guard let stateModel else { return }
        appPhase = .executing
        stateModel.robotState.actionStatus = .executing
        stateModel.robotState.mode = .executing
        appendTranscript(.system, "Executing: \(command.displayName)")
        do {
            try await dispatchCommand(command)
            if case .moveToCartesian = command { appendCartesianResolutionTranscriptIfAvailable() }
            appendTranscript(.feedback, "Completed: \(command.displayName)")
            appPhase = .idle
        } catch {
            stateModel.robotState.actionStatus = .failed
            stateModel.robotState.mode = .faulted
            stateModel.robotState.fault = error.localizedDescription
            appendTranscript(.error, "Execution failed: \(error.localizedDescription)")
            appPhase = .faulted
        }
        pendingCommand = nil
        pendingPreview = nil
        pendingClarification = nil
    }

    func cancelPending() {
        pendingCommand = nil
        pendingPreview = nil
        pendingClarification = nil
        appPhase = .idle
        stateModel?.robotState.actionStatus = .idle
        if stateModel?.robotState.mode == .executing {
            stateModel?.robotState.mode = .idle
        }
        appendTranscript(.system, "Command cancelled")
    }

    func triggerEmergencyStop() async {
        guard let connectionModel, let stateModel else { return }
        appendTranscript(.system, "EMERGENCY STOP")
        do {
            let result = try await connectionModel.robotExecutor.requestEstop(
                reason: "Local user requested emergency stop",
                workspaceSnapshot: spatialModel?.currentWorkspaceSnapshot() ?? GatewayWorkspaceSnapshot(calibrationWorldPosition: nil, pendingRobotGoal: nil, lastConfirmedRobotGoal: nil)
            )
            connectionModel.applyCommandResult(result)
            appendTranscript(.feedback, result.summary)
            connectionModel.remoteBootState = result.estopLatched ? .faulted : .ready
        } catch {
            appendTranscript(.error, "Emergency stop failed: \(error.localizedDescription)")
            connectionModel.remoteBootState = .faulted
            stateModel.robotState.mode = .faulted
            stateModel.robotState.fault = error.localizedDescription
        }
        appPhase = .idle
        pendingCommand = nil
        pendingPreview = nil
        pendingClarification = nil
    }
// MARK: - Perception

    func requestPerceptionSnapshot() async {
        appendTranscript(.system, "Fetching perception snapshot…")
        do {
            guard let connectionModel else {
                appendTranscript(.error, "Perception failed: no connection")
                return
            }
            let snapshot = try await connectionModel.fetchPerceptionSnapshot()
            transcript.append(TranscriptEntry(
                kind: .perception,
                text: snapshot.summary,
                perceptionSnapshot: snapshot
            ))
            if transcript.count > 200 { transcript.removeFirst(transcript.count - 200) }
        } catch {
            appendTranscript(.error, "Perception failed: \(error.localizedDescription)")
        }
    }

// MARK: - Transcript

    func appendTranscript(_ kind: TranscriptEntry.Kind, _ text: String) {
        transcript.append(TranscriptEntry(kind: kind, text: text))
        if transcript.count > 200 { transcript.removeFirst(transcript.count - 200) }
    }

    func appendCartesianResolutionTranscriptIfAvailable() {
        guard let stateModel,
              let resolution = stateModel.lastResolvedCartesianGoal,
              let requested = resolution.requestedPose?.position,
              let resolved = resolution.resolvedPose?.position else { return }
        let resolvedSummary = String(format: "(%.3f, %.3f, %.3f) m", resolved.x, resolved.y, resolved.z)
        if let target = stateModel.lastRequestedCartesianTarget {
            appendTranscript(.system, "Requested \(target.requestSummary) [\(target.frameDisplayName)] → resolved \(resolvedSummary) (\(resolution.reason))")
        } else {
            let requestedSummary = String(format: "(%.3f, %.3f, %.3f) m", requested.x, requested.y, requested.z)
            appendTranscript(.system, "Resolved target \(requestedSummary) → \(resolvedSummary) (\(resolution.reason))")
        }
    }
// MARK: - Private helpers

    private func preview(_ command: RobotCommand) {
        guard let stateModel else { return }
        let safetyResult = stateModel.safetyGate.validate(command, robotState: stateModel.robotState)
        let preview = CommandPreview(
            parsedCommandName: command.displayName,
            targets: extractTargets(from: command),
            validationResult: safetyResult,
            subsystemsTouched: command.subsystemsTouched,
            requiresConfirmation: command.requiresConfirmation,
            canExecute: safetyResult.isAllowed
        )
        pendingClarification = nil
        pendingCommand = command
        pendingPreview = preview
        appendTranscript(.parsedCommand, "Parsed: \(command.displayName)")
        appPhase = command.requiresConfirmation ? .awaitingConfirmation : .executing
        if !command.requiresConfirmation {
            Task { await confirmExecution() }
        }
    }

    private func clearPendingInteractiveState(resetPhase: Bool) {
        pendingCommand = nil
        pendingPreview = nil
        pendingClarification = nil
        if resetPhase, appPhase != .executing {
            appPhase = .idle
        }
    }

    private func dispatchCommand(_ command: RobotCommand) async throws {
        guard let connectionModel, let spatialModel else { return }
        if case .emergencyStop = command {
            await triggerEmergencyStop()
            return
        }
        if case .moveToCartesian(let target) = command {
            stateModel?.lastRequestedCartesianTarget = target
        }
        let result = try await connectionModel.robotExecutor.execute(
            command: command,
            workspaceSnapshot: spatialModel.currentWorkspaceSnapshot()
        )
        connectionModel.applyCommandResult(result)
        appendTranscript(.feedback, result.summary)
    }

    private func extractTargets(from command: RobotCommand) -> [String] {
        switch command {
        case .moveToPreset(let name): return [name]
        case .pickAndPlace(let obj, let dest): return [obj, dest]
        case .moveToCartesian(let target):
            var values = ["Frame: \(target.frameDisplayName)", "Requested: \(target.requestSummary)"]
            if target.source == .voice { values.append("Normalized: \(target.centimeterSummary)") }
            if let safeRange = stateModel?.safeRangeSummary(for: target.referenceFrame) {
                values.append("Safe range: \(safeRange)")
            } else if target.referenceFrame == .toolRelative {
                values.append("Safe range: unavailable until live gripper pose is available")
            }
            return values
        case .captureSnapshot(let cam): return [cam]
        default: return []
        }
    }
// MARK: - AI command parsing

    private func parseCommand(_ utterance: String) async throws -> ParsedUserInput {
        let request = AgentRunRequest(
            sessionKey: "robot-copilot-parser",
            prompt: buildCommandSystemPrompt(for: utterance),
            modelProviderID: selectedProviderID.isEmpty ? nil : selectedProviderID
        )
        let result = try await agentRuntime.run(request)
        return try decodeParsedInput(from: result.output)
    }

    private func buildCommandSystemPrompt(for utterance: String) -> String {
        guard let connectionModel, let stateModel else { return "User: \(utterance)" }
        var lines: [String] = [
            "You are a robot command parser for an SO-101 follower arm.",
            "Parse the user's utterance into exactly one JSON response and respond with ONLY valid JSON.",
            "", "Supported JSON responses:",
        ]
        if connectionModel.supportsHomeCommand { lines.append("- {\"kind\":\"command\",\"command\":\"homeRobot\"}") }
        if connectionModel.supportsCommand("openGripper") { lines.append("- {\"kind\":\"command\",\"command\":\"openGripper\"}") }
        if connectionModel.supportsCommand("closeGripper") { lines.append("- {\"kind\":\"command\",\"command\":\"closeGripper\"}") }
        if !connectionModel.supportedPresetSummary.isEmpty { lines.append("- {\"kind\":\"command\",\"command\":\"moveToPreset\",\"name\":\"<preset_name>\"}") }
        if connectionModel.supportsCommand("moveToCartesian") {
            lines.append("- {\"kind\":\"command\",\"command\":\"moveToCartesian\",\"target\":{\"x\":<float>,\"y\":<float>,\"z\":<float>,\"unit\":\"mm|cm|m\",\"referenceFrame\":\"baseAbsolute|toolRelative\",\"source\":\"voice\"}}")
            lines.append("- {\"kind\":\"clarification\",\"question\":\"Do you mean millimeters, centimeters, or meters?\",\"draftCartesianTarget\":{\"x\":<float>,\"y\":<float>,\"z\":<float>,\"referenceFrame\":\"baseAbsolute|toolRelative\",\"source\":\"voice\"}}")
        }
        if connectionModel.supportsCommand("emergencyStop") { lines.append("- {\"kind\":\"command\",\"command\":\"emergencyStop\"}") }
        lines.append("- {\"kind\":\"unknown\",\"reason\":\"<explanation>\"}")
        lines.append("")
        lines.append("Rules for Cartesian parsing:")
        lines.append("- Accept only metric units mm, cm, and m for spoken Cartesian moves.")
        lines.append("- If the user gives explicit x/y/z coordinates, set referenceFrame to baseAbsolute.")
        lines.append("- If the user says forward/back/left/right/up/down, set referenceFrame to toolRelative.")
        lines.append("- For toolRelative commands use these axis signs: forward=+x, back=-x, left=+y, right=-y, up=+z, down=-z.")
        lines.append("- If a Cartesian command includes distances but omits the unit, return the clarification response and do not guess.")
        lines.append("- For spoken commands, source must always be voice.")
        lines.append("")
        let wb = connectionModel.workspaceBounds
        lines.append("Known presets: \(connectionModel.supportedPresetSummary.isEmpty ? "none" : connectionModel.supportedPresetSummary.joined(separator: ", "))")
        lines.append("Workspace bounds (meters): x [\(wb.xMin), \(wb.xMax)], y [\(wb.yMin), \(wb.yMax)], z [\(wb.zMin), \(wb.zMax)].")
        if let baseRange = stateModel.safeRangeSummary(for: .baseAbsolute) { lines.append("Base-frame safe range (cm): \(baseRange)") }
        if let relativeRange = stateModel.safeRangeSummary(for: .toolRelative) { lines.append("Tool-relative safe delta range from current gripper pose (cm): \(relativeRange)") }
        lines.append("")
        lines.append("User: \(utterance)")
        return lines.joined(separator: "\n")
    }
// MARK: - JSON decoding

    func decodeParsedInput(from json: String) throws -> ParsedUserInput {
        let trimmed = json.trimmingCharacters(in: .whitespacesAndNewlines)
        let jsonStart = trimmed.firstIndex(of: "{") ?? trimmed.startIndex
        let jsonEnd = trimmed.lastIndex(of: "}").map { trimmed.index(after: $0) } ?? trimmed.endIndex
        let jsonString = String(trimmed[jsonStart..<jsonEnd])
        guard let data = jsonString.data(using: .utf8),
              let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else {
            throw OpenClawCoreError.unavailable("Could not parse command JSON from model output")
        }
        let kind = (dict["kind"] as? String)?.lowercased() ?? "command"
        switch kind {
        case "clarification":
            let question = (dict["question"] as? String)?.trimmedNonEmpty ?? "Do you mean millimeters, centimeters, or meters?"
            let draftDict = dict["draftCartesianTarget"] as? [String: Any] ?? dict
            let draft = decodeClarificationDraft(from: draftDict)
            return .clarification(PendingCartesianClarification(question: question, draft: draft))
        case "unknown":
            let reason = (dict["reason"] as? String)?.trimmedNonEmpty ?? "Unknown command"
            throw OpenClawCoreError.unavailable("Unrecognized command: \(reason)")
        default:
            guard let commandName = (dict["command"] as? String)?.trimmedNonEmpty else {
                throw OpenClawCoreError.unavailable("Command JSON did not include a command name")
            }
            switch commandName {
            case "homeRobot": return .command(.homeRobot)
            case "openGripper": return .command(.openGripper)
            case "closeGripper": return .command(.closeGripper)
            case "moveToPreset":
                return .command(.moveToPreset(name: (dict["name"] as? String)?.trimmedNonEmpty ?? "home"))
            case "moveToCartesian":
                if let target = try decodeCartesianTargetIfComplete(from: dict, defaultSource: .voice) {
                    return .command(.moveToCartesian(target: target))
                }
                let draftDict = (dict["target"] as? [String: Any]) ?? dict
                return .clarification(PendingCartesianClarification(
                    question: "Do you mean millimeters, centimeters, or meters?",
                    draft: decodeClarificationDraft(from: draftDict)))
            case "emergencyStop": return .command(.emergencyStop)
            default: throw OpenClawCoreError.unavailable("Unrecognized command: \(commandName)")
            }
        }
    }

    private func decodeCartesianTargetIfComplete(from dict: [String: Any], defaultSource: RobotCartesianSource) throws -> RobotCartesianTarget? {
        let targetDict = (dict["target"] as? [String: Any]) ?? dict
        let x = (targetDict["x"] as? NSNumber)?.doubleValue ?? (targetDict["x"] as? Double) ?? 0.0
        let y = (targetDict["y"] as? NSNumber)?.doubleValue ?? (targetDict["y"] as? Double) ?? 0.0
        let z = (targetDict["z"] as? NSNumber)?.doubleValue ?? (targetDict["z"] as? Double) ?? 0.0
        let referenceFrame = decodeReferenceFrame(raw: targetDict["referenceFrame"] as? String) ?? .baseAbsolute
        let source = decodeSource(raw: targetDict["source"] as? String) ?? defaultSource
        guard let unit = CartesianVoiceCommandSupport.decodeMetricUnit(raw: targetDict["unit"] as? String) else { return nil }
        return RobotCartesianTarget(x: x, y: y, z: z, unit: unit, referenceFrame: referenceFrame, source: source)
    }

    private func decodeClarificationDraft(from dict: [String: Any]) -> PendingCartesianClarificationDraft {
        PendingCartesianClarificationDraft(
            x: (dict["x"] as? NSNumber)?.doubleValue ?? (dict["x"] as? Double) ?? 0.0,
            y: (dict["y"] as? NSNumber)?.doubleValue ?? (dict["y"] as? Double) ?? 0.0,
            z: (dict["z"] as? NSNumber)?.doubleValue ?? (dict["z"] as? Double) ?? 0.0,
            referenceFrame: decodeReferenceFrame(raw: dict["referenceFrame"] as? String) ?? .baseAbsolute,
            source: decodeSource(raw: dict["source"] as? String) ?? .voice
        )
    }

    func resolvePendingClarification(with text: String, clarification: PendingCartesianClarification) async {
        guard let target = CartesianVoiceCommandSupport.finalizeDraft(clarification.draft, with: text) else {
            let remaining = clarification.remainingAttempts - 1
            if remaining > 0 {
                pendingClarification = PendingCartesianClarification(question: clarification.question, draft: clarification.draft, remainingAttempts: remaining)
                appPhase = .awaitingClarification
                appendTranscript(.system, clarification.question)
            } else {
                pendingClarification = nil; pendingCommand = nil; pendingPreview = nil; appPhase = .idle
                appendTranscript(.error, "I still need the Cartesian unit. Please restate the move with millimeters, centimeters, or meters.")
            }
            return
        }
        pendingClarification = nil
        preview(.moveToCartesian(target: target))
    }

    private func decodeReferenceFrame(raw: String?) -> RobotCartesianReferenceFrame? {
        switch raw?.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() {
        case "baseabsolute", "base_absolute", "absolute", "base": return .baseAbsolute
        case "toolrelative", "tool_relative", "relative", "tool": return .toolRelative
        default: return nil
        }
    }

    private func decodeSource(raw: String?) -> RobotCartesianSource? {
        switch raw?.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() {
        case "voice": return .voice
        case "spatial", "stylus": return .spatial
        default: return nil
        }
    }

    // MARK: - Model providers

    private func setupModelProviders() async {
        let env = ProcessInfo.processInfo.environment
        var ids: [String] = []
        if let apiKey = env["OPENAI_API_KEY"], !apiKey.isEmpty {
            let config = OpenAIModelConfig(enabled: true, modelID: "gpt-5.4", apiKey: apiKey)
            let provider = OpenAIModelProvider(configuration: config)
            await agentRuntime.registerModelProvider(provider)
            ids.append(OpenAIModelProvider.providerID)
        }
        let foundationAvailability = FoundationModelsProvider.runtimeAvailability()
        if foundationAvailability.isAvailable {
            let provider = FoundationModelsProvider()
            await agentRuntime.registerModelProvider(provider)
            ids.append(FoundationModelsProvider.providerID)
        }
        availableProviderIDs = ids
        if let first = ids.first {
            selectedProviderID = first
            try? await agentRuntime.setDefaultModelProviderID(first)
        }
    }

    // MARK: - Testing hooks

    func testingDecodeParsedInput(_ json: String) throws -> ParsedUserInput {
        try decodeParsedInput(from: json)
    }

    func testingResolvePendingClarification(with text: String, clarification: PendingCartesianClarification) async {
        await resolvePendingClarification(with: text, clarification: clarification)
    }
}

private extension String {
    var trimmedNonEmpty: String? {
        let t = trimmingCharacters(in: .whitespacesAndNewlines)
        return t.isEmpty ? nil : t
    }
}
