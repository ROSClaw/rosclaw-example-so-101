import Foundation

/// Public facade actor for all ROS2 bridge operations.
/// Implements the tool contract surface defined in the requirements (§7.2).
/// Mirrors the TypeScript RosbridgeTransport in rosclaw/extensions/openclaw-plugin/src/transport/rosbridge/adapter.ts
public actor RosClawBridge {

    // MARK: - Configuration

    public struct Configuration: Sendable {
        public var host: String
        public var port: Int
        public var namespace: String

        public init(host: String = "localhost", port: Int = 9090, namespace: String = "") {
            self.host = host
            self.port = port
            self.namespace = namespace
        }

        public var url: URL {
            URL(string: "ws://\(host):\(port)")!
        }
    }

    // MARK: - State

    private let client: RosClawBridgeClient
    public private(set) var configuration: Configuration
    public private(set) var connectionStatus: ConnectionStatus = .disconnected

    public init(configuration: Configuration = Configuration()) {
        self.configuration = configuration
        self.client = RosClawBridgeClient()
    }

    // MARK: - Connection lifecycle

    public func connect() async throws {
        try await client.connect(to: configuration.url)
        connectionStatus = await client.status
    }

    public func disconnect() async {
        await client.disconnect()
        connectionStatus = .disconnected
    }

    public func updateConfiguration(_ config: Configuration) async {
        configuration = config
    }

    public func connectionStatusStream() -> AsyncStream<ConnectionStatus> {
        return AsyncStream { continuation in
            Task {
                let stream = await client.statusStream()
                for await status in stream {
                    self.connectionStatus = status
                    continuation.yield(status)
                }
                continuation.finish()
            }
        }
    }

    // MARK: - ros2_list_topics

    public func listTopics() async throws -> [TopicInfo] {
        let response = try await callServiceRaw(
            service: "/rosapi/topics",
            type: "rosapi/srv/Topics",
            args: [:],
            timeoutMs: 10_000
        )
        let topics = (response["topics"] as? [String]) ?? []
        let types = (response["types"] as? [String]) ?? []
        return topics.enumerated().map { i, name in
            TopicInfo(name: name, type: types.indices.contains(i) ? types[i] : "")
        }
    }

    // MARK: - ros2_publish

    public func publish(topic: String, type: String, msg: [String: Any]) async throws {
        var message = RosbridgeMessage(op: "publish")
        message.topic = topic
        message.type = type
        message.msg = _AnyJSON(msg)
        try await client.send(message)
    }

    // MARK: - ros2_subscribe_once

    public func subscribeOnce(topic: String, type: String? = nil, timeoutMs: Int = 5_000) async throws -> [String: Any] {
        let id = await client.nextID(prefix: "sub")
        let capturedTopic = topic

        // Use a Sendable enum to carry the result across the continuation boundary
        enum SubscribeResult: Sendable {
            case success([String: _AnyJSON])
            case failure(Error)
        }

        let result: SubscribeResult = await withCheckedContinuation { continuation in
            let once = ContinuationBox<SubscribeResult>(continuation)
            Task {
                await client.addTopicHandler(topic: capturedTopic) { data in
                    guard let msg = try? JSONDecoder().decode(RosbridgeMessage.self, from: data),
                          case .object(let payload) = msg.msg else {
                        once.resume(returning: .failure(BridgeError.decodingFailed))
                        return
                    }
                    once.resume(returning: .success(payload))
                }

                var subMsg = RosbridgeMessage(op: "subscribe", id: id)
                subMsg.topic = capturedTopic
                subMsg.type = type
                try? await self.client.send(subMsg)

                try? await Task.sleep(nanoseconds: UInt64(timeoutMs) * 1_000_000)
                await self.client.removeTopicHandlers(topic: capturedTopic)
                once.resume(returning: .failure(BridgeError.timeout(id: id)))
            }
        }

        switch result {
        case .success(let dict):
            return dict.mapValues { $0.rawValue }
        case .failure(let error):
            throw error
        }
    }

    // MARK: - ros2_service_call

    public func callService(service: String, type: String? = nil, args: [String: Any] = [:], timeoutMs: Int = 30_000) async throws -> [String: Any] {
        try await callServiceRaw(service: service, type: type, args: args, timeoutMs: timeoutMs)
    }

    private func callServiceRaw(service: String, type: String?, args: [String: Any], timeoutMs: Int) async throws -> [String: Any] {
        let id = await client.nextID(prefix: "service")

        let data = try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<Data, Error>) in
            Task {
                await client.registerPending(id: id, continuation: continuation, timeoutMs: timeoutMs)
                var msg = RosbridgeMessage(op: "call_service", id: id)
                msg.service = service
                msg.type = type
                msg.args = _AnyJSON(args)
                try? await self.client.send(msg)
            }
        }

        guard let response = try? JSONDecoder().decode(RosbridgeMessage.self, from: data) else {
            throw BridgeError.decodingFailed
        }
        guard response.result == true else {
            throw BridgeError.serviceCallFailed(service)
        }
        if case .object(let dict) = response.values {
            return dict.mapValues { $0.rawValue }
        }
        return [:]
    }

    // MARK: - ros2_action_goal

    public func sendActionGoal(
        action: String,
        actionType: String,
        args: [String: Any] = [:],
        onFeedback: (@Sendable ([String: Any]) -> Void)? = nil,
        timeoutMs: Int = 120_000
    ) async throws -> [String: Any] {
        let id = await client.nextID(prefix: "action")

        if let onFeedback {
            let feedbackKey = "__action_feedback__\(id)"
            await client.addTopicHandler(topic: feedbackKey) { data in
                guard let msg = try? JSONDecoder().decode(RosbridgeMessage.self, from: data),
                      case .object(let feedback) = msg.feedback else { return }
                onFeedback(feedback.mapValues { $0.rawValue })
            }
        }

        let data = try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<Data, Error>) in
            Task {
                await client.registerPending(id: id, continuation: continuation, timeoutMs: timeoutMs)
                var msg = RosbridgeMessage(op: "send_action_goal", id: id)
                msg.action = action
                msg.action_type = actionType
                msg.args = _AnyJSON(args)
                try? await self.client.send(msg)
            }
        }

        let feedbackKey = "__action_feedback__\(id)"
        await client.removeTopicHandlers(topic: feedbackKey)

        guard let response = try? JSONDecoder().decode(RosbridgeMessage.self, from: data) else {
            throw BridgeError.decodingFailed
        }
        if case .object(let dict) = response.values {
            return dict.mapValues { $0.rawValue }
        }
        return [:]
    }

    // MARK: - ros2_param_get / ros2_param_set

    public func paramGet(node: String, name: String) async throws -> Any {
        let result = try await callServiceRaw(
            service: "\(node)/get_parameters",
            type: "rcl_interfaces/srv/GetParameters",
            args: ["names": [name]],
            timeoutMs: 10_000
        )
        return result
    }

    public func paramSet(node: String, name: String, value: Any) async throws {
        _ = try await callServiceRaw(
            service: "\(node)/set_parameters",
            type: "rcl_interfaces/srv/SetParameters",
            args: ["parameters": [["name": name, "value": value]]],
            timeoutMs: 10_000
        )
    }

    // MARK: - ros2_camera_snapshot

    public func cameraSnapshot(topic: String) async throws -> Data {
        let payload = try await subscribeOnce(topic: topic, timeoutMs: 5_000)
        if let dataArray = payload["data"] as? [Int] {
            return Data(dataArray.map { UInt8(clamping: $0) })
        }
        throw BridgeError.decodingFailed
    }

    // MARK: - ros2_perception_snapshot

    public func perceptionSnapshot(
        topic: String = "/rosclaw/perception",
        timeoutMs: Int = 5_000
    ) async throws -> PerceptionSnapshot {
        let payload = try await subscribeOnce(
            topic: topic,
            type: "rosclaw_msgs/msg/PerceptionSnapshot",
            timeoutMs: timeoutMs
        )

        let summary = (payload["summary"] as? String) ?? ""
        let imageMimeType = (payload["image_mime_type"] as? String) ?? "image/jpeg"

        let imageData: Data
        if let base64 = payload["image_data"] as? String {
            guard let decoded = Data(base64Encoded: base64) else {
                throw BridgeError.decodingFailed
            }
            imageData = decoded
        } else if let intArray = payload["image_data"] as? [Int] {
            imageData = Data(intArray.map { UInt8(clamping: $0) })
        } else {
            imageData = Data()
        }

        let objects: [PerceptionObject] = (payload["objects"] as? [[String: Any]])?.map { obj in
            PerceptionObject(
                label: (obj["label"] as? String) ?? "",
                position: (obj["position"] as? String) ?? "",
                distanceM: (obj["distance_m"] as? Double) ?? 0.0
            )
        } ?? []

        let obstacles: [PerceptionObstacle] = (payload["obstacles"] as? [[String: Any]])?.map { obs in
            PerceptionObstacle(
                label: (obs["label"] as? String) ?? "",
                direction: (obs["direction"] as? String) ?? "",
                distanceM: (obs["distance_m"] as? Double) ?? 0.0
            )
        } ?? []

        let alerts = (payload["alerts"] as? [String]) ?? []

        return PerceptionSnapshot(
            summary: summary,
            imageMimeType: imageMimeType,
            imageData: imageData,
            objects: objects,
            obstacles: obstacles,
            alerts: alerts
        )
    }

    // MARK: - Emergency stop (§10.1 — bypasses agent reasoning)

    public func stopArmTrajectory() async {
        let ns = configuration.namespace.isEmpty ? "/follower" : configuration.namespace
        let topic = "\(ns)/arm_controller/joint_trajectory"
        let emptyTrajectory: [String: Any] = [
            "joint_names": [
                "shoulder_pan", "shoulder_lift", "elbow_flex",
                "wrist_flex", "wrist_roll",
            ],
            "points": [] as [[String: Any]],
        ]
        try? await publish(
            topic: topic,
            type: "trajectory_msgs/msg/JointTrajectory",
            msg: emptyTrajectory
        )
    }

    public func emergencyStop() async {
        await stopArmTrajectory()
    }
}
