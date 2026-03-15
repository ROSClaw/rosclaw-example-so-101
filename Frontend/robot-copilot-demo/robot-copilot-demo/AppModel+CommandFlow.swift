import Foundation
import OpenClawAgents
import OpenClawCore
import RosClawBridge

extension AppModel {
    func handleInput(_ text: String) async {
        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return }
        appendTranscript(.userInput, trimmed)

        guard remoteBootState == .ready else {
            appendTranscript(.error, "The Gateway executor is not ready yet.")
            return
        }

        let lower = trimmed.lowercased()
        if lower.contains("stop") || lower.contains("estop") || lower.contains("emergency") {
            await triggerEmergencyStop()
            return
        }

        appPhase = .previewing

        do {
            let command = try await parseCommand(trimmed)
            let safetyResult = safetyGate.validate(command, robotState: robotState)
            let preview = CommandPreview(
                parsedCommandName: command.displayName,
                targets: extractTargets(from: command),
                validationResult: safetyResult,
                subsystemsTouched: command.subsystemsTouched,
                requiresConfirmation: command.requiresConfirmation,
                canExecute: safetyResult.isAllowed
            )
            pendingCommand = command
            pendingPreview = preview
            appendTranscript(.parsedCommand, "Parsed: \(command.displayName)")
            appPhase = command.requiresConfirmation ? .awaitingConfirmation : .executing
            if !command.requiresConfirmation {
                await confirmExecution()
            }
        } catch {
            appendTranscript(.error, "Parse error: \(error.localizedDescription)")
            appPhase = .idle
        }
    }

    func confirmExecution() async {
        guard let command = pendingCommand else { return }
        guard let preview = pendingPreview, preview.canExecute else {
            appendTranscript(.error, "Cannot execute: \(pendingPreview?.validationResult.blockReason ?? "unknown reason")")
            cancelPending()
            return
        }

        appPhase = .executing
        robotState.actionStatus = .executing
        robotState.mode = .executing
        appendTranscript(.system, "Executing: \(command.displayName)")

        do {
            try await dispatchCommand(command)
            if case .moveToCartesian = command {
                appendCartesianResolutionTranscriptIfAvailable()
            }
            appendTranscript(.feedback, "Completed: \(command.displayName)")
            appPhase = .idle
        } catch {
            robotState.actionStatus = .failed
            robotState.mode = .faulted
            robotState.fault = error.localizedDescription
            appendTranscript(.error, "Execution failed: \(error.localizedDescription)")
            appPhase = .faulted
        }

        pendingCommand = nil
        pendingPreview = nil
    }

    func cancelPending() {
        pendingCommand = nil
        pendingPreview = nil
        appPhase = .idle
        robotState.actionStatus = .idle
        if robotState.mode == .executing {
            robotState.mode = .idle
        }
        appendTranscript(.system, "Command cancelled")
    }

    func triggerEmergencyStop() async {
        appendTranscript(.system, "EMERGENCY STOP")
        do {
            let result = try await robotExecutor.requestEstop(
                reason: "Local user requested emergency stop",
                workspaceSnapshot: currentWorkspaceSnapshot()
            )
            applyCommandResult(result)
            appendTranscript(.feedback, result.summary)
            remoteBootState = result.estopLatched ? .faulted : .ready
        } catch {
            appendTranscript(.error, "Emergency stop failed: \(error.localizedDescription)")
            remoteBootState = .faulted
            robotState.mode = .faulted
            robotState.fault = error.localizedDescription
        }

        appPhase = .idle
        pendingCommand = nil
        pendingPreview = nil
    }

    func sendCartesianGoal(x: Double, y: Double, z: Double) async {
        let goal = SIMD3<Float>(Float(x), Float(y), Float(z))
        let command = RobotCommand.moveToCartesian(x: x, y: y, z: z)
        let safetyResult = safetyGate.validate(command, robotState: robotState)
        guard safetyResult.isAllowed else {
            appendTranscript(.error, safetyResult.blockReason ?? "Cartesian goal is not safe to execute.")
            appPhase = .idle
            return
        }
        confirmSpatialGoal(goal)

        appendTranscript(.parsedCommand, "Spatial target: \(command.displayName)")
        appPhase = .executing
        robotState.actionStatus = .executing
        robotState.mode = .executing

        do {
            try await dispatchCommand(command)
            appendCartesianResolutionTranscriptIfAvailable()
            appendTranscript(.feedback, "Completed: \(command.displayName)")
            appPhase = .idle
        } catch {
            robotState.actionStatus = .failed
            robotState.mode = .faulted
            robotState.fault = error.localizedDescription
            appendTranscript(.error, "Cartesian goal failed: \(error.localizedDescription)")
            appPhase = .faulted
        }
    }

    private func parseCommand(_ utterance: String) async throws -> RobotCommand {
        let request = AgentRunRequest(
            sessionKey: "robot-copilot-parser",
            prompt: buildCommandSystemPrompt(for: utterance),
            modelProviderID: selectedProviderID.isEmpty ? nil : selectedProviderID
        )

        let result = try await agentRuntime.run(request)
        return try decodeCommand(from: result.output)
    }

    private func buildCommandSystemPrompt(for utterance: String) -> String {
        var lines: [String] = [
            "You are a robot command parser for an SO-101 follower arm.",
            "Parse the user's utterance into exactly one JSON command object and respond with ONLY valid JSON.",
            "",
            "Supported command objects:",
        ]

        if supportsHomeCommand {
            lines.append("- {\"command\": \"homeRobot\"}")
        }
        if supportsCommand("openGripper") {
            lines.append("- {\"command\": \"openGripper\"}")
        }
        if supportsCommand("closeGripper") {
            lines.append("- {\"command\": \"closeGripper\"}")
        }
        if !supportedPresetSummary.isEmpty {
            lines.append("- {\"command\": \"moveToPreset\", \"name\": \"<preset_name>\"}")
        }
        if supportsCommand("moveToCartesian") {
            lines.append("- {\"command\": \"moveToCartesian\", \"x\": <float>, \"y\": <float>, \"z\": <float>}")
        }
        if supportsCommand("emergencyStop") {
            lines.append("- {\"command\": \"emergencyStop\"}")
        }

        lines.append("")
        lines.append("Known presets: \(supportedPresetSummary.isEmpty ? "none" : supportedPresetSummary.joined(separator: ", "))")
        lines.append(
            "Workspace bounds (meters): x [\(workspaceBounds.xMin), \(workspaceBounds.xMax)], y [\(workspaceBounds.yMin), \(workspaceBounds.yMax)], z [\(workspaceBounds.zMin), \(workspaceBounds.zMax)]."
        )
        lines.append("")
        lines.append("If the utterance requests an unsupported capability, is ambiguous, or does not match a supported command, respond with:")
        lines.append("{\"command\": \"unknown\", \"reason\": \"<explanation>\"}")
        lines.append("")
        lines.append("User: \(utterance)")

        return lines.joined(separator: "\n")
    }

    private func decodeCommand(from json: String) throws -> RobotCommand {
        let trimmed = json.trimmingCharacters(in: .whitespacesAndNewlines)
        let jsonStart = trimmed.firstIndex(of: "{") ?? trimmed.startIndex
        let jsonEnd = trimmed.lastIndex(of: "}").map { trimmed.index(after: $0) } ?? trimmed.endIndex
        let jsonString = String(trimmed[jsonStart..<jsonEnd])

        guard let data = jsonString.data(using: .utf8),
              let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let commandName = dict["command"] as? String else {
            throw OpenClawCoreError.unavailable("Could not parse command JSON from model output")
        }

        switch commandName {
        case "homeRobot":
            return .homeRobot
        case "openGripper":
            return .openGripper
        case "closeGripper":
            return .closeGripper
        case "moveToPreset":
            let name = dict["name"] as? String ?? "home"
            return .moveToPreset(name: name)
        case "moveToCartesian":
            let x = dict["x"] as? Double ?? 0.0
            let y = dict["y"] as? Double ?? 0.0
            let z = dict["z"] as? Double ?? 0.0
            return .moveToCartesian(x: x, y: y, z: z)
        case "emergencyStop":
            return .emergencyStop
        default:
            let reason = dict["reason"] as? String ?? "Unknown command"
            throw OpenClawCoreError.unavailable("Unrecognized command: \(reason)")
        }
    }

    private func dispatchCommand(_ command: RobotCommand) async throws {
        if case .emergencyStop = command {
            await triggerEmergencyStop()
            return
        }

        let result = try await robotExecutor.execute(
            command: command,
            workspaceSnapshot: currentWorkspaceSnapshot()
        )
        applyCommandResult(result)
        appendTranscript(.feedback, result.summary)
    }

    private func supportsCommand(_ name: String) -> Bool {
        operationRegistrySummary.contains(name)
    }

    private var supportsHomeCommand: Bool {
        supportedPresetSummary.contains { $0.caseInsensitiveCompare("home") == .orderedSame }
    }

    private func extractTargets(from command: RobotCommand) -> [String] {
        switch command {
        case .moveToPreset(let name):
            return [name]
        case .pickAndPlace(let obj, let dest):
            return [obj, dest]
        case .moveToCartesian(let x, let y, let z):
            return [String(format: "x=%.3f", x), String(format: "y=%.3f", y), String(format: "z=%.3f", z)]
        case .captureSnapshot(let cam):
            return [cam]
        default:
            return []
        }
    }

    private func appendCartesianResolutionTranscriptIfAvailable() {
        guard let resolution = lastResolvedCartesianGoal,
              let requested = resolution.requestedPose?.position,
              let resolved = resolution.resolvedPose?.position
        else {
            return
        }

        let requestedSummary = String(
            format: "(%.3f, %.3f, %.3f)",
            requested.x,
            requested.y,
            requested.z
        )
        let resolvedSummary = String(
            format: "(%.3f, %.3f, %.3f)",
            resolved.x,
            resolved.y,
            resolved.z
        )
        appendTranscript(
            .system,
            "Resolved target \(requestedSummary) → \(resolvedSummary) (\(resolution.reason))"
        )
    }
}
