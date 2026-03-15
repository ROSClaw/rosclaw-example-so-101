import Foundation
import OpenClawProtocol

struct WebSocketSessionBox: Sendable {}

struct GatewayConnectOptions: Sendable, Equatable {
    let role: String
    let scopes: [String]
    let caps: [String]
    let commands: [String]
    let permissions: [String: Bool]
    let clientId: String
    let clientMode: String
    let clientDisplayName: String?
    let includeDeviceIdentity: Bool

    nonisolated init(
        role: String,
        scopes: [String],
        caps: [String],
        commands: [String],
        permissions: [String: Bool],
        clientId: String,
        clientMode: String,
        clientDisplayName: String?,
        includeDeviceIdentity: Bool = true
    ) {
        self.role = role
        self.scopes = scopes
        self.caps = caps
        self.commands = commands
        self.permissions = permissions
        self.clientId = clientId
        self.clientMode = clientMode
        self.clientDisplayName = clientDisplayName
        self.includeDeviceIdentity = includeDeviceIdentity
    }
}

enum OpenClawNodeErrorCode: String, Codable, Sendable {
    case notPaired = "NOT_PAIRED"
    case unauthorized = "UNAUTHORIZED"
    case backgroundUnavailable = "NODE_BACKGROUND_UNAVAILABLE"
    case invalidRequest = "INVALID_REQUEST"
    case unavailable = "UNAVAILABLE"
}

struct OpenClawNodeError: Error, Codable, Sendable, Equatable {
    let code: OpenClawNodeErrorCode
    let message: String
    let retryable: Bool?
    let retryAfterMs: Int?

    nonisolated init(
        code: OpenClawNodeErrorCode,
        message: String,
        retryable: Bool? = nil,
        retryAfterMs: Int? = nil
    ) {
        self.code = code
        self.message = message
        self.retryable = retryable
        self.retryAfterMs = retryAfterMs
    }
}

struct BridgeInvokeRequest: Codable, Sendable {
    let type: String
    let id: String
    let command: String
    let paramsJSON: String?

    nonisolated init(type: String = "invoke", id: String, command: String, paramsJSON: String? = nil) {
        self.type = type
        self.id = id
        self.command = command
        self.paramsJSON = paramsJSON
    }
}

struct BridgeInvokeResponse: Codable, Sendable {
    let type: String
    let id: String
    let ok: Bool
    let payloadJSON: String?
    let error: OpenClawNodeError?

    nonisolated init(
        type: String = "invoke-res",
        id: String,
        ok: Bool,
        payloadJSON: String? = nil,
        error: OpenClawNodeError? = nil
    ) {
        self.type = type
        self.id = id
        self.ok = ok
        self.payloadJSON = payloadJSON
        self.error = error
    }
}

private struct GatewayHelloOk: Decodable {
    let serverName: String?
    let canvasHostURL: String?
    let mainSessionKey: String?
    let auth: [String: AnyCodable]
    let policy: [String: AnyCodable]

    enum CodingKeys: String, CodingKey {
        case serverName
        case canvasHostURL = "canvasHostUrl"
        case mainSessionKey
        case auth
        case policy
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.serverName = try container.decodeIfPresent(String.self, forKey: .serverName)
        self.canvasHostURL = try container.decodeIfPresent(String.self, forKey: .canvasHostURL)
        self.mainSessionKey = try container.decodeIfPresent(String.self, forKey: .mainSessionKey)
        self.auth = try container.decodeIfPresent([String: AnyCodable].self, forKey: .auth) ?? [:]
        self.policy = try container.decodeIfPresent([String: AnyCodable].self, forKey: .policy) ?? [:]
    }
}

private struct GatewayResolvedConnectAuth: Sendable {
    let authToken: String?
    let authBootstrapToken: String?
    let authPassword: String?
    let signatureToken: String?

    nonisolated init(
        authToken: String?,
        authBootstrapToken: String?,
        authPassword: String?,
        signatureToken: String?
    ) {
        self.authToken = authToken
        self.authBootstrapToken = authBootstrapToken
        self.authPassword = authPassword
        self.signatureToken = signatureToken
    }
}

struct GatewayConnectIdentitySnapshot: Sendable, Equatable {
    let platform: String
    let displayName: String
    let instanceId: String
    let deviceFamily: String
    let modelIdentifier: String?
    let locale: String
    let userAgent: String

    nonisolated static func current(displayNameOverride: String?) -> GatewayConnectIdentitySnapshot {
        let displayName = displayNameOverride?.gatewayTrimmedNonEmpty ?? GatewayInstanceIdentity.displayName
        let locale = Locale.preferredLanguages.first ?? Locale.current.identifier
        return GatewayConnectIdentitySnapshot(
            platform: GatewayInstanceIdentity.platformString,
            displayName: displayName,
            instanceId: GatewayInstanceIdentity.instanceId,
            deviceFamily: GatewayInstanceIdentity.deviceFamily,
            modelIdentifier: GatewayInstanceIdentity.modelIdentifier,
            locale: locale,
            userAgent: ProcessInfo.processInfo.operatingSystemVersionString
        )
    }
}

enum GatewayConnectFrameEncoder {
    nonisolated static func encode(
        requestID: String,
        nonce: String,
        options: GatewayConnectOptions,
        appVersion: String,
        identity: GatewayConnectIdentitySnapshot,
        deviceIdentity: GatewayDeviceIdentity?,
        authToken: String?,
        authBootstrapToken: String?,
        authPassword: String?,
        signatureToken: String?
    ) throws -> Data {
        var client: [String: Any] = [
            "id": options.clientId,
            "displayName": identity.displayName,
            "version": appVersion,
            "platform": identity.platform,
            "deviceFamily": identity.deviceFamily,
            "mode": options.clientMode,
            "instanceId": identity.instanceId,
        ]
        if let modelIdentifier = identity.modelIdentifier {
            client["modelIdentifier"] = modelIdentifier
        }

        var auth: [String: Any]?
        if let authToken {
            auth = ["token": authToken]
        } else if let authBootstrapToken {
            auth = ["bootstrapToken": authBootstrapToken]
        } else if let authPassword {
            auth = ["password": authPassword]
        } else {
            auth = nil
        }

        var device: [String: Any]?
        let signedAtMs = Int(Date().timeIntervalSince1970 * 1000)
        if options.includeDeviceIdentity,
           let deviceIdentity,
           let signature = GatewayDeviceIdentityStore.signPayload(
               GatewayDeviceAuthPayload.buildV3(
                   deviceId: deviceIdentity.deviceId,
                   clientId: options.clientId,
                   clientMode: options.clientMode,
                   role: options.role,
                   scopes: options.scopes,
                   signedAtMs: signedAtMs,
                   token: signatureToken,
                   nonce: nonce,
                   platform: identity.platform,
                   deviceFamily: identity.deviceFamily
               ),
               identity: deviceIdentity
           ),
           let publicKey = GatewayDeviceIdentityStore.publicKeyBase64URL(deviceIdentity)
        {
            device = [
                "id": deviceIdentity.deviceId,
                "publicKey": publicKey,
                "signature": signature,
                "signedAt": signedAtMs,
                "nonce": nonce,
            ]
        } else {
            device = nil
        }

        var params: [String: Any] = [
            "minProtocol": GATEWAY_PROTOCOL_VERSION,
            "maxProtocol": GATEWAY_PROTOCOL_VERSION,
            "client": client,
            "caps": options.caps,
            "role": options.role,
            "scopes": options.scopes,
            "locale": identity.locale,
            "userAgent": identity.userAgent,
        ]
        if !options.commands.isEmpty {
            params["commands"] = options.commands
        }
        if !options.permissions.isEmpty {
            params["permissions"] = options.permissions
        }
        if let device {
            params["device"] = device
        }
        if let auth {
            params["auth"] = auth
        }

        let request: [String: Any] = [
            "type": "req",
            "id": requestID,
            "method": "connect",
            "params": params,
        ]
        return try JSONSerialization.data(withJSONObject: request)
    }
}

private struct GatewayNodeInvokeRequestPayload: Decodable {
    let id: String
    let nodeId: String
    let command: String
    let paramsJSON: String?
    let params: [String: AnyCodable]?
    let timeoutMs: Int?

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.id = try container.decode(String.self, forKey: .id)
        self.nodeId = try container.decode(String.self, forKey: .nodeId)
        self.command = try container.decode(String.self, forKey: .command)
        self.paramsJSON = try container.decodeIfPresent(String.self, forKey: .paramsJSON)
        self.params = try container.decodeIfPresent([String: AnyCodable].self, forKey: .params)
        self.timeoutMs = try container.decodeIfPresent(Int.self, forKey: .timeoutMs)
    }

    private enum CodingKeys: String, CodingKey {
        case id
        case nodeId
        case command
        case paramsJSON
        case params
        case timeoutMs
    }
}

private enum GatewaySessionError: LocalizedError {
    case notConnected
    case connectChallengeMissing
    case connectChallengeTimeout
    case invalidResponse(String)
    case remoteFailure(code: String?, message: String)
    case requestTimeout(String)

    var errorDescription: String? {
        switch self {
        case .notConnected:
            return "Gateway is not connected."
        case .connectChallengeMissing:
            return "Gateway connect challenge did not include a nonce."
        case .connectChallengeTimeout:
            return "Gateway connect challenge timed out."
        case .invalidResponse(let message):
            return message
        case .remoteFailure(_, let message):
            return message
        case .requestTimeout(let requestID):
            return "Gateway request timed out (\(requestID))."
        }
    }
}

actor GatewayNodeSession {
    private let encoder = JSONEncoder()
    private let decoder = JSONDecoder()
    private let defaultTickIntervalMs = 30_000.0
    private let keepaliveIntervalNs: UInt64 = 15 * 1_000_000_000

    private var urlSession: URLSession?
    private var socket: URLSessionWebSocketTask?
    private var receiveTask: Task<Void, Never>?
    private var watchdogTask: Task<Void, Never>?
    private var keepaliveTask: Task<Void, Never>?
    private var reconnectTask: Task<Void, Never>?
    private var connectWaiters: [CheckedContinuation<Void, Error>] = []
    private var pendingResponses: [String: CheckedContinuation<ResponseFrame, Error>] = [:]

    private var activeConfig: StoredConfig?
    private var activeConnectionKey: String?
    private var onConnected: (@Sendable () async -> Void)?
    private var onDisconnected: (@Sendable (String) async -> Void)?
    private var onInvoke: (@Sendable (BridgeInvokeRequest) async -> BridgeInvokeResponse)?
    private var onEvent: (@Sendable (String, AnyCodable?) async -> Void)?
    private var isConnecting = false
    private var isConnected = false
    private var shouldReconnect = false
    private var tickIntervalMs = 30_000.0
    private var lastTick = Date()
    private var reconnectBackoffNs: UInt64 = 500_000_000

    func connect(
        url: URL,
        token: String?,
        bootstrapToken: String?,
        password: String?,
        connectOptions: GatewayConnectOptions,
        sessionBox _: WebSocketSessionBox?,
        onConnected: @escaping @Sendable () async -> Void,
        onDisconnected: @escaping @Sendable (String) async -> Void,
        onEvent: (@Sendable (String, AnyCodable?) async -> Void)? = nil,
        onInvoke: @escaping @Sendable (BridgeInvokeRequest) async -> BridgeInvokeResponse
    ) async throws {
        let nextConfig = StoredConfig(
            url: url,
            token: token?.gatewayTrimmedNonEmpty,
            bootstrapToken: bootstrapToken?.gatewayTrimmedNonEmpty,
            password: password?.gatewayTrimmedNonEmpty,
            options: connectOptions
        )
        let nextKey = nextConfig.connectionKey
        let needsReconnect =
            !self.isConnected ||
            self.activeConfig != nextConfig ||
            self.activeConnectionKey != nextKey

        self.activeConfig = nextConfig
        self.activeConnectionKey = nextKey
        self.onConnected = onConnected
        self.onDisconnected = onDisconnected
        self.onEvent = onEvent
        self.onInvoke = onInvoke
        self.shouldReconnect = true

        if !needsReconnect {
            return
        }

        try await self.establishConnectionIfNeeded(forceReconnect: true)
    }

    func disconnect() async {
        self.shouldReconnect = false
        self.reconnectTask?.cancel()
        self.reconnectTask = nil
        await self.closeCurrentConnection(reason: nil, notify: false)
        self.activeConfig = nil
        self.activeConnectionKey = nil
        self.onConnected = nil
        self.onDisconnected = nil
        self.onEvent = nil
        self.onInvoke = nil
    }

    func request(method: String, paramsJSON: String?, timeoutSeconds: Int = 15) async throws -> Data {
        try await self.establishConnectionIfNeeded(forceReconnect: false)
        guard let socket else {
            throw GatewaySessionError.notConnected
        }

        let params = try Self.parseParams(paramsJSON)
        let requestID = UUID().uuidString
        let frame = RequestFrame(
            type: "req",
            id: requestID,
            method: method,
            params: params.map(AnyCodable.init)
        )
        let payload = try self.encoder.encode(frame)

        let response: ResponseFrame = try await withCheckedThrowingContinuation { continuation in
            self.pendingResponses[requestID] = continuation

            Task { [weak self] in
                guard let self else { return }
                let timeoutNs = UInt64(max(timeoutSeconds, 1)) * 1_000_000_000
                try? await Task.sleep(nanoseconds: timeoutNs)
                await self.timeoutRequest(requestID: requestID)
            }

            Task {
                do {
                    try await socket.send(.data(payload))
                } catch {
                    let waiter = self.removePendingResponse(for: requestID)
                    waiter?.resume(throwing: error)
                    await self.handleConnectionFailure(reason: error.localizedDescription)
                }
            }
        }

        return try self.encodeResponsePayload(response)
    }

    private func fireAndForget(method: String, params: [String: AnyCodable]) async throws {
        try await self.establishConnectionIfNeeded(forceReconnect: false)
        guard let socket else {
            throw GatewaySessionError.notConnected
        }

        let frame = RequestFrame(
            type: "req",
            id: UUID().uuidString,
            method: method,
            params: AnyCodable(params)
        )
        let payload = try self.encoder.encode(frame)
        try await socket.send(.data(payload))
    }

    private func establishConnectionIfNeeded(forceReconnect: Bool) async throws {
        guard let config = self.activeConfig else {
            throw GatewaySessionError.notConnected
        }

        if self.isConnected, !forceReconnect {
            return
        }

        if self.isConnecting {
            try await withCheckedThrowingContinuation { continuation in
                self.connectWaiters.append(continuation)
            }
            return
        }

        self.isConnecting = true
        defer { self.isConnecting = false }

        if forceReconnect {
            await self.closeCurrentConnection(reason: nil, notify: false)
        }

        let session = URLSession(configuration: .default)
        let socket = session.webSocketTask(with: config.url)
        socket.maximumMessageSize = 16 * 1024 * 1024
        socket.resume()

        do {
            let nonce = try await self.waitForConnectChallenge(from: socket)
            let connectRequestID = UUID().uuidString
            let data = try self.makeConnectFrameData(
                requestID: connectRequestID,
                nonce: nonce,
                configuration: config
            )
            try await socket.send(.data(data))
            let response = try await self.waitForConnectResponse(
                requestID: connectRequestID,
                from: socket
            )

            guard response.ok else {
                throw GatewaySessionError.remoteFailure(
                    code: response.error?.code.rawValue,
                    message: response.error?.message ?? "Gateway connect failed."
                )
            }

            if let payload = response.payload {
                let payloadData = try self.encoder.encode(payload)
                if let hello = try? self.decoder.decode(GatewayHelloOk.self, from: payloadData) {
                    if let interval = hello.policy["tickIntervalMs"]?.doubleValue, interval > 0 {
                        self.tickIntervalMs = interval
                    } else {
                        self.tickIntervalMs = self.defaultTickIntervalMs
                    }
                    self.persistDeviceTokenIfPresent(hello: hello, configuration: config)
                } else {
                    self.tickIntervalMs = self.defaultTickIntervalMs
                }
            } else {
                self.tickIntervalMs = self.defaultTickIntervalMs
            }

            self.urlSession = session
            self.socket = socket
            self.isConnected = true
            self.lastTick = Date()
            self.reconnectBackoffNs = 500_000_000
            self.startReceiveLoop(with: socket)
            self.startKeepalive()
            self.startWatchdog()
            self.resumeConnectWaiters()
            await self.onConnected?()
        } catch {
            socket.cancel(with: .goingAway, reason: nil)
            session.invalidateAndCancel()
            self.resumeConnectWaiters(throwing: error)
            throw error
        }
    }

    private func waitForConnectChallenge(from socket: URLSessionWebSocketTask) async throws -> String {
        let deadline = Date().addingTimeInterval(6)
        while Date() < deadline {
            let message = try await socket.receive()
            guard let frame = try self.decodeGatewayFrame(from: message) else {
                continue
            }
            switch frame {
            case .event(let event):
                guard event.event == "connect.challenge" else { continue }
                guard let nonce = event.payload?.objectValue?["nonce"]?.stringValue?.gatewayTrimmedNonEmpty else {
                    throw GatewaySessionError.connectChallengeMissing
                }
                return nonce
            case .res:
                continue
            case .req:
                continue
            }
        }
        throw GatewaySessionError.connectChallengeTimeout
    }

    private func waitForConnectResponse(
        requestID: String,
        from socket: URLSessionWebSocketTask
    ) async throws -> ResponseFrame {
        let deadline = Date().addingTimeInterval(6)
        while Date() < deadline {
            let message = try await socket.receive()
            guard let frame = try self.decodeGatewayFrame(from: message) else {
                continue
            }
            switch frame {
            case .res(let response) where response.id == requestID:
                return response
            case .event(let event):
                if event.event == "tick" {
                    self.lastTick = Date()
                }
            default:
                continue
            }
        }
        throw GatewaySessionError.invalidResponse("Gateway connect response timed out.")
    }

    private func makeConnectFrameData(
        requestID: String,
        nonce: String,
        configuration: StoredConfig
    ) throws -> Data {
        let deviceIdentity = configuration.options.includeDeviceIdentity ? GatewayDeviceIdentityStore.loadOrCreate() : nil
        let resolvedAuth = self.resolveConnectAuth(configuration: configuration, identity: deviceIdentity)
        let identitySnapshot = GatewayConnectIdentitySnapshot.current(
            displayNameOverride: configuration.options.clientDisplayName
        )
        let appVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "dev"
        return try GatewayConnectFrameEncoder.encode(
            requestID: requestID,
            nonce: nonce,
            options: configuration.options,
            appVersion: appVersion,
            identity: identitySnapshot,
            deviceIdentity: deviceIdentity,
            authToken: resolvedAuth.authToken,
            authBootstrapToken: resolvedAuth.authBootstrapToken,
            authPassword: resolvedAuth.authPassword,
            signatureToken: resolvedAuth.signatureToken
        )
    }

    private func startReceiveLoop(with socket: URLSessionWebSocketTask) {
        self.receiveTask?.cancel()
        self.receiveTask = Task { [weak self] in
            guard let self else { return }
            do {
                while !Task.isCancelled {
                    let message = try await socket.receive()
                    await self.handleIncoming(message)
                }
            } catch {
                await self.handleConnectionFailure(reason: error.localizedDescription)
            }
        }
    }

    private func handleIncoming(_ message: URLSessionWebSocketTask.Message) async {
        guard let frame = try? self.decodeGatewayFrame(from: message) else {
            return
        }

        switch frame {
        case .res(let response):
            let waiter = self.pendingResponses.removeValue(forKey: response.id)
            waiter?.resume(returning: response)
        case .event(let event):
            if event.event == "tick" {
                self.lastTick = Date()
            } else if event.event == "node.invoke.request" {
                await self.handleNodeInvokeRequest(event)
            } else if let onEvent {
                await onEvent(event.event, event.payload)
            }
        case .req:
            break
        }
    }

    private func handleNodeInvokeRequest(_ event: EventFrame) async {
        guard let payload = event.payload else {
            return
        }

        do {
            let requestPayload = try self.decodeNodeInvokePayload(from: payload)
            guard let onInvoke else { return }
            let paramsJSON = try self.normalizedParamsJSON(
                explicitJSON: requestPayload.paramsJSON,
                paramsObject: requestPayload.params
            )
            let request = BridgeInvokeRequest(
                id: requestPayload.id,
                command: requestPayload.command,
                paramsJSON: paramsJSON
            )
            let response = await Self.invokeWithTimeout(
                request: request,
                timeoutMs: requestPayload.timeoutMs,
                onInvoke: onInvoke
            )
            try await self.sendNodeInvokeResult(
                requestID: requestPayload.id,
                nodeID: requestPayload.nodeId,
                response: response
            )
        } catch {
            return
        }
    }

    private func sendNodeInvokeResult(
        requestID: String,
        nodeID: String,
        response: BridgeInvokeResponse
    ) async throws {
        var params: [String: AnyCodable] = [
            "id": AnyCodable(requestID),
            "nodeId": AnyCodable(nodeID),
            "ok": AnyCodable(response.ok),
        ]
        if let payloadJSON = response.payloadJSON {
            params["payloadJSON"] = AnyCodable(payloadJSON)
        }
        if let error = response.error {
            params["error"] = AnyCodable([
                "code": AnyCodable(error.code.rawValue),
                "message": AnyCodable(error.message),
            ])
        }
        try await self.fireAndForget(method: "node.invoke.result", params: params)
    }

    private func normalizedParamsJSON(
        explicitJSON: String?,
        paramsObject: [String: AnyCodable]?
    ) throws -> String? {
        if let explicitJSON = explicitJSON?.gatewayTrimmedNonEmpty {
            return explicitJSON
        }
        guard let paramsObject else {
            return nil
        }
        let data = try self.encoder.encode(paramsObject)
        return String(data: data, encoding: .utf8)
    }

    private func decodeNodeInvokePayload(from payload: AnyCodable) throws -> GatewayNodeInvokeRequestPayload {
        let data = try self.encoder.encode(payload)
        return try self.decoder.decode(GatewayNodeInvokeRequestPayload.self, from: data)
    }

    private func encodeResponsePayload(_ response: ResponseFrame) throws -> Data {
        if response.ok == false {
            throw GatewaySessionError.remoteFailure(
                code: response.error?.code.rawValue,
                message: response.error?.message ?? "Gateway request failed."
            )
        }
        if let payload = response.payload {
            return try self.encoder.encode(payload)
        }
        return Data("null".utf8)
    }

    private func resolveConnectAuth(
        configuration: StoredConfig,
        identity: GatewayDeviceIdentity?
    ) -> GatewayResolvedConnectAuth {
        let explicitToken = configuration.token?.gatewayTrimmedNonEmpty
        let explicitBootstrapToken = configuration.bootstrapToken?.gatewayTrimmedNonEmpty
        let explicitPassword = configuration.password?.gatewayTrimmedNonEmpty
        let storedToken: String?
        if configuration.options.includeDeviceIdentity, let identity {
            storedToken = GatewayDeviceAuthStore.loadToken(
                deviceId: identity.deviceId,
                role: configuration.options.role
            )?.token.gatewayTrimmedNonEmpty
        } else {
            storedToken = nil
        }
        let authToken =
            explicitToken ??
                (configuration.options.includeDeviceIdentity &&
                    explicitPassword == nil &&
                    (explicitBootstrapToken == nil || storedToken != nil) ? storedToken : nil)
        let authBootstrapToken = authToken == nil ? explicitBootstrapToken : nil
        return GatewayResolvedConnectAuth(
            authToken: authToken,
            authBootstrapToken: authBootstrapToken,
            authPassword: explicitPassword,
            signatureToken: authToken ?? authBootstrapToken
        )
    }

    private func persistDeviceTokenIfPresent(hello: GatewayHelloOk, configuration: StoredConfig) {
        guard configuration.options.includeDeviceIdentity else {
            return
        }
        guard let deviceToken = hello.auth["deviceToken"]?.stringValue?.gatewayTrimmedNonEmpty else {
            return
        }
        let identity = GatewayDeviceIdentityStore.loadOrCreate()
        let role = hello.auth["role"]?.stringValue?.gatewayTrimmedNonEmpty ?? configuration.options.role
        let scopes = hello.auth["scopes"]?.arrayValue?.compactMap { $0.stringValue?.gatewayTrimmedNonEmpty } ?? []
        GatewayDeviceAuthStore.storeToken(
            deviceId: identity.deviceId,
            role: role,
            token: deviceToken,
            scopes: scopes
        )
    }

    private func handleConnectionFailure(reason: String) async {
        if !self.isConnected && self.receiveTask == nil {
            return
        }
        await self.closeCurrentConnection(reason: reason, notify: true)
        guard self.shouldReconnect else {
            return
        }
        self.scheduleReconnect()
    }

    private func closeCurrentConnection(reason: String?, notify: Bool) async {
        let receiveTask = self.receiveTask
        let watchdogTask = self.watchdogTask
        let keepaliveTask = self.keepaliveTask
        let socket = self.socket
        let session = self.urlSession

        self.receiveTask = nil
        self.watchdogTask = nil
        self.keepaliveTask = nil
        self.socket = nil
        self.urlSession = nil
        self.isConnected = false
        self.tickIntervalMs = self.defaultTickIntervalMs

        receiveTask?.cancel()
        watchdogTask?.cancel()
        keepaliveTask?.cancel()
        socket?.cancel(with: .goingAway, reason: nil)
        session?.invalidateAndCancel()

        let pending = self.pendingResponses
        self.pendingResponses.removeAll()
        for (_, continuation) in pending {
            continuation.resume(throwing: GatewaySessionError.notConnected)
        }

        if notify, let reason {
            await self.onDisconnected?(reason)
        }
    }

    private func scheduleReconnect() {
        guard self.reconnectTask == nil else {
            return
        }
        self.reconnectTask = Task { [weak self] in
            guard let self else { return }
            while await self.shouldAttemptReconnect() {
                let backoff = await self.nextReconnectBackoff()
                try? await Task.sleep(nanoseconds: backoff)
                do {
                    try await self.establishConnectionIfNeeded(forceReconnect: false)
                    await self.clearReconnectTask()
                    return
                } catch {
                    continue
                }
            }
            await self.clearReconnectTask()
        }
    }

    private func startKeepalive() {
        self.keepaliveTask?.cancel()
        self.keepaliveTask = Task { [weak self] in
            guard let self else { return }
            while await self.shouldAttemptReconnect() {
                try? await Task.sleep(nanoseconds: self.keepaliveIntervalNs)
                guard let socket = await self.socketIfConnected() else {
                    continue
                }
                do {
                    try await Self.sendPing(on: socket)
                } catch {
                    await self.handleConnectionFailure(reason: error.localizedDescription)
                    return
                }
            }
        }
    }

    private func startWatchdog() {
        self.watchdogTask?.cancel()
        self.watchdogTask = Task { [weak self] in
            guard let self else { return }
            while await self.shouldAttemptReconnect() {
                let sleepNs = await self.currentWatchdogSleepNs()
                try? await Task.sleep(nanoseconds: sleepNs)
                let isStale = await self.isTickWatchStale()
                if isStale {
                    await self.handleConnectionFailure(reason: "Gateway tick timed out.")
                    return
                }
            }
        }
    }

    private func isTickWatchStale() -> Bool {
        guard self.isConnected else {
            return false
        }
        let elapsedMs = Date().timeIntervalSince(self.lastTick) * 1000
        return elapsedMs > (self.tickIntervalMs * 2)
    }

    private func currentWatchdogSleepNs() -> UInt64 {
        UInt64(max(self.tickIntervalMs, 1_000) * 1_000_000)
    }

    private func nextReconnectBackoff() -> UInt64 {
        let next = self.reconnectBackoffNs
        self.reconnectBackoffNs = min(self.reconnectBackoffNs * 2, 30_000_000_000)
        return next
    }

    private func shouldAttemptReconnect() -> Bool {
        self.shouldReconnect && self.activeConfig != nil
    }

    private func clearReconnectTask() {
        self.reconnectTask = nil
    }

    private func timeoutRequest(requestID: String) {
        let waiter = self.pendingResponses.removeValue(forKey: requestID)
        waiter?.resume(throwing: GatewaySessionError.requestTimeout(requestID))
    }

    private func removePendingResponse(for requestID: String) -> CheckedContinuation<ResponseFrame, Error>? {
        self.pendingResponses.removeValue(forKey: requestID)
    }

    private func resumeConnectWaiters(throwing error: Error? = nil) {
        let waiters = self.connectWaiters
        self.connectWaiters.removeAll()
        for waiter in waiters {
            if let error {
                waiter.resume(throwing: error)
            } else {
                waiter.resume(returning: ())
            }
        }
    }

    private func socketIfConnected() -> URLSessionWebSocketTask? {
        guard self.isConnected else {
            return nil
        }
        return self.socket
    }

    private func decodeGatewayFrame(from message: URLSessionWebSocketTask.Message) throws -> GatewayFrame? {
        let data: Data
        switch message {
        case .data(let bytes):
            data = bytes
        case .string(let string):
            guard let encoded = string.data(using: .utf8) else {
                return nil
            }
            data = encoded
        @unknown default:
            return nil
        }
        return try self.decoder.decode(GatewayFrame.self, from: data)
    }

    private static func parseParams(_ paramsJSON: String?) throws -> [String: AnyCodable]? {
        guard let paramsJSON = paramsJSON?.gatewayTrimmedNonEmpty else {
            return nil
        }
        guard let data = paramsJSON.data(using: .utf8) else {
            throw GatewaySessionError.invalidResponse("Gateway params were not UTF-8.")
        }
        let raw = try JSONSerialization.jsonObject(with: data)
        guard let dictionary = raw as? [String: Any] else {
            return nil
        }
        return try dictionary.mapValues(Self.makeAnyCodable)
    }

    private static func makeAnyCodable(_ value: Any) throws -> AnyCodable {
        switch value {
        case is NSNull:
            return try JSONDecoder().decode(AnyCodable.self, from: Data("null".utf8))
        case let bool as Bool:
            return AnyCodable(bool)
        case let int as Int:
            return AnyCodable(int)
        case let int as Int64:
            return AnyCodable(Int(int))
        case let number as Double:
            return AnyCodable(number)
        case let number as Float:
            return AnyCodable(Double(number))
        case let string as String:
            return AnyCodable(string)
        case let array as [Any]:
            return AnyCodable(try array.map(Self.makeAnyCodable))
        case let dictionary as [String: Any]:
            return AnyCodable(try dictionary.mapValues(Self.makeAnyCodable))
        default:
            throw GatewaySessionError.invalidResponse("Unsupported gateway JSON value: \(type(of: value))")
        }
    }

    private static func sendPing(on socket: URLSessionWebSocketTask) async throws {
        try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<Void, Error>) in
            socket.sendPing { error in
                if let error {
                    continuation.resume(throwing: error)
                } else {
                    continuation.resume(returning: ())
                }
            }
        }
    }

    private static func invokeWithTimeout(
        request: BridgeInvokeRequest,
        timeoutMs: Int?,
        onInvoke: @escaping @Sendable (BridgeInvokeRequest) async -> BridgeInvokeResponse
    ) async -> BridgeInvokeResponse {
        let timeout = max(timeoutMs ?? 30_000, 0)
        guard timeout > 0 else {
            return await onInvoke(request)
        }

        final class InvokeBox: @unchecked Sendable {
            private let lock = NSLock()
            private var continuation: CheckedContinuation<BridgeInvokeResponse, Never>?
            private var didResume = false

            func setContinuation(_ continuation: CheckedContinuation<BridgeInvokeResponse, Never>) {
                self.lock.lock()
                defer { self.lock.unlock() }
                self.continuation = continuation
            }

            func resume(_ response: BridgeInvokeResponse) {
                let continuation: CheckedContinuation<BridgeInvokeResponse, Never>?
                self.lock.lock()
                if self.didResume {
                    self.lock.unlock()
                    return
                }
                self.didResume = true
                continuation = self.continuation
                self.continuation = nil
                self.lock.unlock()
                continuation?.resume(returning: response)
            }
        }

        let box = InvokeBox()
        var timeoutTask: Task<Void, Never>?
        var invokeTask: Task<Void, Never>?
        defer {
            timeoutTask?.cancel()
            invokeTask?.cancel()
        }

        return await withCheckedContinuation { continuation in
            box.setContinuation(continuation)

            invokeTask = Task.detached {
                let response = await onInvoke(request)
                box.resume(response)
            }

            timeoutTask = Task.detached {
                try? await Task.sleep(nanoseconds: UInt64(timeout) * 1_000_000)
                box.resume(
                    BridgeInvokeResponse(
                        id: request.id,
                        ok: false,
                        error: OpenClawNodeError(
                            code: .unavailable,
                            message: "node invoke timed out"
                        )
                    )
                )
            }
        }
    }
}

private struct StoredConfig: Equatable {
    let url: URL
    let token: String?
    let bootstrapToken: String?
    let password: String?
    let options: GatewayConnectOptions

    nonisolated var connectionKey: String {
        let permissions = self.options.permissions.keys.sorted().map {
            "\($0)=\(self.options.permissions[$0] == true ? "1" : "0")"
        }.joined(separator: ",")
        let parts = [
            self.url.absoluteString,
            self.token ?? "",
            self.bootstrapToken ?? "",
            self.password ?? "",
            self.options.role,
            self.options.scopes.joined(separator: ","),
            self.options.caps.joined(separator: ","),
            self.options.commands.joined(separator: ","),
            self.options.clientId,
            self.options.clientMode,
            self.options.clientDisplayName ?? "",
            self.options.includeDeviceIdentity ? "1" : "0",
            permissions,
        ]
        return parts.joined(separator: "|")
    }
}

private extension String {
    nonisolated var gatewayTrimmedNonEmpty: String? {
        let trimmed = self.trimmingCharacters(in: .whitespacesAndNewlines)
        return trimmed.isEmpty ? nil : trimmed
    }
}
