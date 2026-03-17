import Foundation

/// Low-level WebSocket actor that speaks the rosbridge v2 protocol.
/// Mirrors the TypeScript `RosbridgeClient` in rosclaw/extensions/openclaw-plugin/src/transport/rosbridge/client.ts
actor RosClawBridgeClient {

    // MARK: - State

    private var task: URLSessionWebSocketTask?
    private var session: URLSession
    private(set) var status: ConnectionStatus = .disconnected
    private var idCounter: Int = 0
    private var pendingRequests: [String: CheckedContinuation<Data, Error>] = [:]
    private var topicHandlers: [String: [(Data) -> Void]] = [:]
    private var statusContinuations: [UUID: AsyncStream<ConnectionStatus>.Continuation] = [:]
    private var reconnectTask: Task<Void, Never>?
    private var intentionalClose = false
    private var reconnectAttempts = 0
    private let maxReconnectAttempts = 10
    private let reconnectInterval: TimeInterval = 3.0
    private var currentURL: URL?

    init() {
        self.session = URLSession(configuration: .default)
    }

    // MARK: - Connection

    func connect(to url: URL) async throws {
        guard status != .connected else { return }
        intentionalClose = false
        currentURL = url
        setStatus(.connecting)

        let wsTask = session.webSocketTask(with: url)
        self.task = wsTask
        wsTask.resume()

        // Wait for connection by attempting a ping with timeout
        try await withThrowingTaskGroup(of: Void.self) { group in
            group.addTask {
                try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<Void, Error>) in
                    wsTask.sendPing { error in
                        if let error {
                            continuation.resume(throwing: error)
                        } else {
                            continuation.resume()
                        }
                    }
                }
            }
            group.addTask {
                try await Task.sleep(nanoseconds: 10_000_000_000) // 10s timeout
                throw BridgeError.connectionTimeout
            }
            try await group.next()
            group.cancelAll()
        }

        setStatus(.connected)
        reconnectAttempts = 0
        startReceiveLoop()
    }

    func disconnect() async {
        intentionalClose = true
        reconnectTask?.cancel()
        reconnectTask = nil
        task?.cancel(with: .normalClosure, reason: nil)
        task = nil
        rejectAllPending(BridgeError.disconnected)
        setStatus(.disconnected)
    }

    // MARK: - Send

    func send(_ message: RosbridgeMessage) async throws {
        guard let task, status == .connected else {
            throw BridgeError.notConnected
        }
        let data = try JSONEncoder().encode(message)
        let string = String(data: data, encoding: .utf8) ?? "{}"
        try await task.send(.string(string))
    }

    // MARK: - Pending request tracking

    func nextID(prefix: String = "bridge") -> String {
        idCounter += 1
        return "\(prefix)_\(idCounter)"
    }

    func registerPending(id: String, continuation: CheckedContinuation<Data, Error>, timeoutMs: Int = 30_000) {
        pendingRequests[id] = continuation
        let capturedID = id
        Task { [weak self] in
            try? await Task.sleep(nanoseconds: UInt64(timeoutMs) * 1_000_000)
            await self?.timeoutPending(id: capturedID)
        }
    }

    private func timeoutPending(id: String) {
        guard let cont = pendingRequests.removeValue(forKey: id) else { return }
        cont.resume(throwing: BridgeError.timeout(id: id))
    }

    func resolvePending(id: String, data: Data) {
        pendingRequests.removeValue(forKey: id)?.resume(returning: data)
    }

    func rejectPending(id: String, error: Error) {
        pendingRequests.removeValue(forKey: id)?.resume(throwing: error)
    }

    private func rejectAllPending(_ error: Error) {
        for cont in pendingRequests.values { cont.resume(throwing: error) }
        pendingRequests.removeAll()
    }

    // MARK: - Topic subscriptions

    func addTopicHandler(topic: String, handler: @escaping (Data) -> Void) {
        topicHandlers[topic, default: []].append(handler)
    }

    func removeTopicHandlers(topic: String) {
        topicHandlers.removeValue(forKey: topic)
    }

    // MARK: - Status stream

    func statusStream() -> AsyncStream<ConnectionStatus> {
        AsyncStream { continuation in
            let id = UUID()
            statusContinuations[id] = continuation
            continuation.onTermination = { [weak self] _ in
                Task { await self?.removeStatusContinuation(id: id) }
            }
        }
    }

    private func removeStatusContinuation(id: UUID) {
        statusContinuations.removeValue(forKey: id)
    }

    private func setStatus(_ newStatus: ConnectionStatus) {
        status = newStatus
        for cont in statusContinuations.values { cont.yield(newStatus) }
    }

    // MARK: - Receive loop

    private func startReceiveLoop() {
        guard task != nil else { return }
        Task { [weak self] in
            while let self = self {
                let currentTask = await self.currentTask()
                guard let currentTask else { break }
                do {
                    let message = try await currentTask.receive()
                    await self.handleMessage(message)
                } catch {
                    await self.handleDisconnect()
                    break
                }
            }
        }
    }

    private func currentTask() -> URLSessionWebSocketTask? { task }

    private func handleMessage(_ message: URLSessionWebSocketTask.Message) {
        let data: Data
        switch message {
        case .string(let str): data = Data(str.utf8)
        case .data(let d): data = d
        @unknown default: return
        }

        guard let msg = try? JSONDecoder().decode(RosbridgeMessage.self, from: data) else { return }

        switch msg.op {
        case "publish":
            if let topic = msg.topic {
                let handlers = topicHandlers[topic] ?? []
                for handler in handlers { handler(data) }
            }
        case "service_response":
            if let id = msg.id { resolvePending(id: id, data: data) }
        case "action_result":
            if let id = msg.id { resolvePending(id: id, data: data) }
        case "action_feedback":
            if let id = msg.id {
                let feedbackKey = "__action_feedback__\(id)"
                let handlers = topicHandlers[feedbackKey] ?? []
                for handler in handlers { handler(data) }
            }
        default: break
        }
    }

    private func handleDisconnect() {
        task = nil
        rejectAllPending(BridgeError.disconnected)
        setStatus(.disconnected)
        guard !intentionalClose else { return }
        scheduleReconnect()
    }

    private func scheduleReconnect() {
        guard reconnectAttempts < maxReconnectAttempts, let url = currentURL else { return }
        reconnectAttempts += 1
        let delay = min(reconnectInterval * pow(2.0, Double(reconnectAttempts - 1)), 30.0)
        reconnectTask = Task { [weak self] in
            try? await Task.sleep(nanoseconds: UInt64(delay * 1_000_000_000))
            guard let self, !Task.isCancelled else { return }
            try? await self.connect(to: url)
        }
    }
}

// MARK: - Errors

public enum BridgeError: Error, LocalizedError, Sendable {
    case notConnected
    case connectionTimeout
    case disconnected
    case timeout(id: String)
    case serviceCallFailed(String)
    case decodingFailed

    public var errorDescription: String? {
        switch self {
        case .notConnected: return "Not connected to rosbridge"
        case .connectionTimeout: return "Connection to rosbridge timed out"
        case .disconnected: return "Disconnected from rosbridge"
        case .timeout(let id): return "Request \(id) timed out"
        case .serviceCallFailed(let msg): return "Service call failed: \(msg)"
        case .decodingFailed: return "Failed to decode rosbridge response"
        }
    }
}
