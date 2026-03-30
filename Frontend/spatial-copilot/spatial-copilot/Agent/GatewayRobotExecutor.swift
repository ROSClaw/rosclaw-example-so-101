import Foundation
import OpenClawKit
import OpenClawProtocol
import RosClawBridge

enum GatewayRobotBootState: String, Sendable {
    case readingConfig = "Reading Config"
    case connectingOperator = "Connecting Gateway"
    case connectingNode = "Registering Node"
    case selectingAgent = "Selecting Agent"
    case syncingState = "Syncing State"
    case ready = "Ready"
    case faulted = "Faulted"
}

enum GatewayVisionOSClientID {
    static let primary = "openclaw-ios"
    static let legacy = "moltbot-ios"

    nonisolated static func normalized(_ raw: String?) -> String? {
        guard let raw = raw?.trimmedNonEmpty else {
            return nil
        }
        let normalized = raw.lowercased()
        switch normalized {
        case "openclaw-ios", "moltbot-ios":
            return normalized
        default:
            return nil
        }
    }

    nonisolated static func legacyFallback(currentClientID: String, error: Error) -> String? {
        guard currentClientID.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() == "openclaw-ios" else {
            return nil
        }
        let message = error.localizedDescription.lowercased()
        guard message.contains("invalid connect params"), message.contains("/client/id") else {
            return nil
        }
        return "moltbot-ios"
    }
}

enum GatewayRuntimeNodeIdentity {
    nonisolated static func resolve(
        includeDeviceIdentity: Bool,
        clientID: String,
        deviceID: String?
    ) -> String {
        guard includeDeviceIdentity, let deviceID = deviceID?.trimmedNonEmpty else {
            return clientID
        }
        return deviceID
    }
}

enum GatewayOperatorConnectScopes {
    nonisolated static let oracle = [
        "operator.admin",
        "operator.read",
        "operator.write",
        "operator.approvals",
        "operator.pairing",
    ]
}

struct GatewayRobotEndpointConfiguration: Sendable {
    let gatewayURL: URL
    let authToken: String?
    let bootstrapToken: String?
    let password: String?
    let executorAgentIDHint: String?
    let executorSessionKey: String
    let operatorClientID: String
    let nodeClientID: String
    let displayName: String?

    static func from(environment: [String: String]) throws -> GatewayRobotEndpointConfiguration {
        if let gatewayURL = environment["OPENCLAW_GATEWAY_URL"]?.trimmedNonEmpty {
            guard let url = GatewayRobotEndpointConfiguration.normalizeGatewayURL(gatewayURL) else {
                throw GatewayRobotExecutorError.invalidConfiguration(
                    "OPENCLAW_GATEWAY_URL is invalid: \(gatewayURL)")
            }
            let operatorClientID =
                GatewayVisionOSClientID.normalized(environment["OPENCLAW_OPERATOR_CLIENT_ID"])
                ?? GatewayVisionOSClientID.primary
            let nodeClientID =
                GatewayVisionOSClientID.normalized(environment["OPENCLAW_NODE_CLIENT_ID"])
                ?? GatewayVisionOSClientID.normalized(environment["OPENCLAW_NODE_ID"])
                ?? operatorClientID
            return GatewayRobotEndpointConfiguration(
                gatewayURL: url,
                authToken: environment["OPENCLAW_GATEWAY_TOKEN"]?.trimmedNonEmpty,
                bootstrapToken: environment["OPENCLAW_GATEWAY_BOOTSTRAP_TOKEN"]?.trimmedNonEmpty,
                password: environment["OPENCLAW_GATEWAY_PASSWORD"]?.trimmedNonEmpty,
                executorAgentIDHint: environment["OPENCLAW_ROSCLAW_AGENT_ID"]?.trimmedNonEmpty,
                executorSessionKey: environment["OPENCLAW_ROSCLAW_SESSION_KEY"]?.trimmedNonEmpty ?? "visionos-robot",
                operatorClientID: operatorClientID,
                nodeClientID: nodeClientID,
                displayName: environment["OPENCLAW_NODE_DISPLAY_NAME"]?.trimmedNonEmpty ?? "VisionOS Copilot"
            )
        }

        guard let host = environment["ROSCLAW_AGENT_IP"]?.trimmedNonEmpty else {
            throw GatewayRobotExecutorError.invalidConfiguration(
                "Set OPENCLAW_GATEWAY_URL or ROSCLAW_AGENT_IP to connect the visionOS app to the gateway.")
        }

        let port = Int(environment["ROSCLAW_AGENT_PORT"]?.trimmedNonEmpty ?? "") ?? 18789
        guard let url = URL(string: "ws://\(host):\(port)") else {
            throw GatewayRobotExecutorError.invalidConfiguration(
                "ROSCLAW_AGENT_IP / ROSCLAW_AGENT_PORT produced an invalid gateway URL.")
        }

        let operatorClientID =
            GatewayVisionOSClientID.normalized(environment["OPENCLAW_OPERATOR_CLIENT_ID"])
            ?? GatewayVisionOSClientID.primary
        let nodeClientID =
            GatewayVisionOSClientID.normalized(environment["OPENCLAW_NODE_CLIENT_ID"])
            ?? GatewayVisionOSClientID.normalized(environment["OPENCLAW_NODE_ID"])
            ?? operatorClientID
        return GatewayRobotEndpointConfiguration(
            gatewayURL: url,
            authToken: environment["ROSCLAW_AGENT_AUTH_TOKEN"]?.trimmedNonEmpty,
            bootstrapToken: nil,
            password: nil,
            executorAgentIDHint: environment["OPENCLAW_ROSCLAW_AGENT_ID"]?.trimmedNonEmpty,
            executorSessionKey: environment["OPENCLAW_ROSCLAW_SESSION_KEY"]?.trimmedNonEmpty ?? "visionos-robot",
            operatorClientID: operatorClientID,
            nodeClientID: nodeClientID,
            displayName: environment["OPENCLAW_NODE_DISPLAY_NAME"]?.trimmedNonEmpty ?? "VisionOS Copilot"
        )
    }

    private static func normalizeGatewayURL(_ raw: String) -> URL? {
        let trimmed = raw.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return nil }
        if let direct = URL(string: trimmed), direct.scheme != nil {
            return direct
        }
        return URL(string: "wss://\(trimmed)")
    }
}

struct GatewayFollowerState: Codable, Sendable {
    let jointOrder: [String]
    let jointPositions: [String: Double]
    let gripperPosition: Double?
    let gripperOpenFraction: Double?
    let toolPose: RobotToolPose?
    let toolPoseStatus: GatewayToolPoseStatus?
    let workspaceBounds: WorkspaceBounds?
    let namedPoseNames: [String]
    let currentNamedPose: String?
    let lastResolvedCartesianGoal: GatewayResolvedCartesianGoal?

    enum CodingKeys: String, CodingKey {
        case jointOrder = "joint_order"
        case jointPositions = "joint_positions"
        case gripperPosition = "gripper_position"
        case gripperOpenFraction = "gripper_open_fraction"
        case toolPose = "tool_pose"
        case toolPoseStatus = "tool_pose_status"
        case workspaceBounds = "workspace_bounds"
        case namedPoseNames = "named_pose_names"
        case currentNamedPose = "current_named_pose"
        case lastResolvedCartesianGoal = "last_resolved_cartesian_goal"
    }

    nonisolated init(
        jointOrder: [String],
        jointPositions: [String: Double],
        gripperPosition: Double?,
        gripperOpenFraction: Double?,
        toolPose: RobotToolPose?,
        toolPoseStatus: GatewayToolPoseStatus? = nil,
        workspaceBounds: WorkspaceBounds? = nil,
        namedPoseNames: [String] = [],
        currentNamedPose: String? = nil,
        lastResolvedCartesianGoal: GatewayResolvedCartesianGoal? = nil
    ) {
        self.jointOrder = jointOrder
        self.jointPositions = jointPositions
        self.gripperPosition = gripperPosition
        self.gripperOpenFraction = gripperOpenFraction
        self.toolPose = toolPose
        self.toolPoseStatus = toolPoseStatus
        self.workspaceBounds = workspaceBounds
        self.namedPoseNames = namedPoseNames
        self.currentNamedPose = currentNamedPose
        self.lastResolvedCartesianGoal = lastResolvedCartesianGoal
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.jointOrder = try container.decodeIfPresent([String].self, forKey: .jointOrder) ?? []
        self.jointPositions = try container.decodeIfPresent([String: Double].self, forKey: .jointPositions) ?? [:]
        self.gripperPosition = try container.decodeIfPresent(Double.self, forKey: .gripperPosition)
        self.gripperOpenFraction = try container.decodeIfPresent(Double.self, forKey: .gripperOpenFraction)
        self.toolPose = try container.decodeIfPresent(RobotToolPose.self, forKey: .toolPose)
        self.toolPoseStatus = try container.decodeIfPresent(GatewayToolPoseStatus.self, forKey: .toolPoseStatus)
        self.workspaceBounds = try container.decodeIfPresent(
            GatewayWorkspaceBoundsPayload.self,
            forKey: .workspaceBounds
        )?.workspaceBounds
        self.namedPoseNames = try container.decodeIfPresent([String].self, forKey: .namedPoseNames) ?? []
        self.currentNamedPose = try container.decodeIfPresent(String.self, forKey: .currentNamedPose)
        self.lastResolvedCartesianGoal = try container.decodeIfPresent(
            GatewayResolvedCartesianGoal.self,
            forKey: .lastResolvedCartesianGoal
        )
    }
}

struct GatewayToolPoseStatus: Codable, Sendable, Equatable {
    let stage: String
    let summary: String
    let jointStateReceived: Bool
    let jointCount: Int
    let robotDescriptionReceived: Bool
    let kinematicsReady: Bool
    let missingJointNames: [String]
    let baseFrame: String?
    let toolFrame: String?
    let updatedAt: String?

    enum CodingKeys: String, CodingKey {
        case stage
        case summary
        case jointStateReceived = "joint_state_received"
        case jointCount = "joint_count"
        case robotDescriptionReceived = "robot_description_received"
        case kinematicsReady = "kinematics_ready"
        case missingJointNames = "missing_joint_names"
        case baseFrame = "base_frame"
        case toolFrame = "tool_frame"
        case updatedAt = "updated_at"
    }

    nonisolated init(
        stage: String,
        summary: String,
        jointStateReceived: Bool = false,
        jointCount: Int = 0,
        robotDescriptionReceived: Bool = false,
        kinematicsReady: Bool = false,
        missingJointNames: [String] = [],
        baseFrame: String? = nil,
        toolFrame: String? = nil,
        updatedAt: String? = nil
    ) {
        self.stage = stage
        self.summary = summary
        self.jointStateReceived = jointStateReceived
        self.jointCount = jointCount
        self.robotDescriptionReceived = robotDescriptionReceived
        self.kinematicsReady = kinematicsReady
        self.missingJointNames = missingJointNames
        self.baseFrame = baseFrame
        self.toolFrame = toolFrame
        self.updatedAt = updatedAt
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.stage = try container.decodeIfPresent(String.self, forKey: .stage) ?? "unknown"
        self.summary = try container.decodeIfPresent(String.self, forKey: .summary) ?? "Tool-pose status unavailable."
        self.jointStateReceived = try container.decodeIfPresent(Bool.self, forKey: .jointStateReceived) ?? false
        self.jointCount = try container.decodeIfPresent(Int.self, forKey: .jointCount) ?? 0
        self.robotDescriptionReceived = try container.decodeIfPresent(Bool.self, forKey: .robotDescriptionReceived) ?? false
        self.kinematicsReady = try container.decodeIfPresent(Bool.self, forKey: .kinematicsReady) ?? false
        self.missingJointNames = try container.decodeIfPresent([String].self, forKey: .missingJointNames) ?? []
        self.baseFrame = try container.decodeIfPresent(String.self, forKey: .baseFrame)
        self.toolFrame = try container.decodeIfPresent(String.self, forKey: .toolFrame)
        self.updatedAt = try container.decodeIfPresent(String.self, forKey: .updatedAt)
    }
}

struct GatewayResolvedCartesianGoal: Codable, Sendable, Equatable {
    let accepted: Bool
    let reason: String
    let requestedPose: RobotToolPose?
    let resolvedPose: RobotToolPose?
    let jointPositions: [Double]
    let positionErrorM: Double?
    let orientationErrorRad: Double?
    let executed: Bool?

    enum CodingKeys: String, CodingKey {
        case accepted
        case reason
        case requestedPose = "requested_pose"
        case resolvedPose = "resolved_pose"
        case jointPositions = "joint_positions"
        case positionErrorM = "position_error_m"
        case orientationErrorRad = "orientation_error_rad"
        case executed
    }
}

struct GatewayRobotCapabilities: Sendable {
    let supportedCommands: [String]
    let supportedPresets: [String]
    let workspaceBounds: WorkspaceBounds

    nonisolated static func fromOracle(
        supportedCommands: [String],
        supportedPresets: [String],
        workspaceBounds: WorkspaceBounds?
    ) -> GatewayRobotCapabilities {
        let normalizedPresets = Array(
            Set(
                supportedPresets
                    .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
                    .filter { !$0.isEmpty }
            )
        ).sorted()
        var normalizedCommands = Set(
            supportedCommands.map(Self.normalizeOracleCommand)
                .filter { !$0.isEmpty }
        )
        normalizedCommands.insert("emergencyStop")
        if !normalizedPresets.isEmpty {
            normalizedCommands.insert("moveToPreset")
        }
        if normalizedPresets.contains(where: { $0.caseInsensitiveCompare("home") == .orderedSame }) {
            normalizedCommands.insert("homeRobot")
        }
        return GatewayRobotCapabilities(
            supportedCommands: Array(normalizedCommands).sorted(),
            supportedPresets: normalizedPresets,
            workspaceBounds: workspaceBounds ?? .followerDefault
        )
    }

    nonisolated static let followerDefault = GatewayRobotCapabilities(
        supportedCommands: ["openGripper", "closeGripper", "moveToCartesian", "emergencyStop"],
        supportedPresets: [],
        workspaceBounds: .followerDefault
    )

    nonisolated func enriching(with state: GatewayFollowerState?) -> GatewayRobotCapabilities {
        guard let state else {
            return self
        }
        let mergedPresets = Array(
            Set(
                (supportedPresets + state.namedPoseNames)
                    .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
                    .filter { !$0.isEmpty }
            )
        ).sorted()
        var mergedCommands = Set(supportedCommands)
        if !mergedPresets.isEmpty {
            mergedCommands.insert("moveToPreset")
        }
        if mergedPresets.contains(where: { $0.caseInsensitiveCompare("home") == .orderedSame }) {
            mergedCommands.insert("homeRobot")
        }
        return GatewayRobotCapabilities(
            supportedCommands: Array(mergedCommands).sorted(),
            supportedPresets: mergedPresets,
            workspaceBounds: state.workspaceBounds ?? workspaceBounds
        )
    }

    nonisolated private static func normalizeOracleCommand(_ raw: String) -> String {
        switch raw.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() {
        case "open_gripper", "opengripper":
            return "openGripper"
        case "close_gripper", "closegripper":
            return "closeGripper"
        case "move_to_cartesian", "movetocartesian":
            return "moveToCartesian"
        case "move_to_preset", "movetopreset":
            return "moveToPreset"
        case "home_robot", "homerobot":
            return "homeRobot"
        case "emergency_stop", "set_estop", "estop", "emergencystop":
            return "emergencyStop"
        default:
            return raw.trimmingCharacters(in: .whitespacesAndNewlines)
        }
    }
}

struct GatewayRobotSnapshot: Sendable {
    let executorAgentID: String
    let executorSessionKey: String
    let nodeID: String
    let capabilities: GatewayRobotCapabilities
    let state: GatewayFollowerState?
    let stateSummary: String?
    let estopLatched: Bool
}

struct GatewayRobotCommandResult: Sendable {
    let intentID: String
    let ok: Bool
    let summary: String
    let state: GatewayFollowerState?
    let errorCode: String?
    let estopLatched: Bool
}

struct GatewayWorkspaceSnapshot: Sendable {
    let stage: String?
    let calibrationWorldPosition: SIMD3<Float>?
    let calibrationSurfaceNormal: SIMD3<Float>?
    let calibrationGripperWorldPoint: SIMD3<Float>?
    let pendingRobotGoal: SIMD3<Float>?
    let pendingRobotGoalWorldPoint: SIMD3<Float>?
    let lastConfirmedRobotGoal: SIMD3<Float>?

    init(
        stage: String? = nil,
        calibrationWorldPosition: SIMD3<Float>?,
        calibrationSurfaceNormal: SIMD3<Float>? = nil,
        calibrationGripperWorldPoint: SIMD3<Float>? = nil,
        pendingRobotGoal: SIMD3<Float>?,
        pendingRobotGoalWorldPoint: SIMD3<Float>? = nil,
        lastConfirmedRobotGoal: SIMD3<Float>?
    ) {
        self.stage = stage
        self.calibrationWorldPosition = calibrationWorldPosition
        self.calibrationSurfaceNormal = calibrationSurfaceNormal
        self.calibrationGripperWorldPoint = calibrationGripperWorldPoint
        self.pendingRobotGoal = pendingRobotGoal
        self.pendingRobotGoalWorldPoint = pendingRobotGoalWorldPoint
        self.lastConfirmedRobotGoal = lastConfirmedRobotGoal
    }

    nonisolated var promptSnippet: String {
        var lines: [String] = []
        if let stage {
            lines.append("- onboardingStage: \(stage)")
        }
        if let calibrationWorldPosition {
            lines.append("- calibrationWorldPosition: \(Self.format(vector: calibrationWorldPosition))")
        }
        if let calibrationSurfaceNormal {
            lines.append("- calibrationSurfaceNormal: \(Self.format(vector: calibrationSurfaceNormal))")
        }
        if let calibrationGripperWorldPoint {
            lines.append("- calibrationGripperWorldPoint: \(Self.format(vector: calibrationGripperWorldPoint))")
        }
        if let pendingRobotGoal {
            lines.append("- pendingRobotGoal: \(Self.format(vector: pendingRobotGoal))")
        }
        if let pendingRobotGoalWorldPoint {
            lines.append("- pendingRobotGoalWorldPoint: \(Self.format(vector: pendingRobotGoalWorldPoint))")
        }
        if let lastConfirmedRobotGoal {
            lines.append("- lastConfirmedRobotGoal: \(Self.format(vector: lastConfirmedRobotGoal))")
        }
        return lines.isEmpty ? "- unavailable" : lines.joined(separator: "\n")
    }

    nonisolated private static func format(vector: SIMD3<Float>) -> String {
        String(format: "{\"x\": %.4f, \"y\": %.4f, \"z\": %.4f}", vector.x, vector.y, vector.z)
    }
}

enum GatewayRosclawPromptOracle {
    nonisolated static var manifestResolutionGuidance: String {
        """
        Always resolve live robot interfaces from the robot capability manifest before acting:
        1. Call `ros2_list_topics` to inspect the available topics, services, and actions.
        2. Prefer the exact SO-101 follower endpoints when they are present.
        3. If an exact SO-101 endpoint is absent, use the best manifest candidate with the same final path component inside the active robot namespace.
        4. Do not claim success for an endpoint you did not find in the manifest or through the tool result.
        """
    }

    nonisolated static var stateResolutionGuidance: String {
        """
        Resolve robot state in this order:
        1. Prefer `/follower/agent_state` or any topic whose final path component is `agent_state` and whose type is `std_msgs/msg/String`.
        2. If an `agent_state` topic is available, use `ros2_subscribe_once`, parse the JSON string from `message.data`, and copy the parsed object into `state`.
        3. Otherwise prefer `/follower/joint_states` or any topic whose final path component is `joint_states` and whose type is `sensor_msgs/msg/JointState`.
        4. If a `joint_states` topic is available, use `ros2_subscribe_once` and build a partial `state` object:
           - `joint_order` from `message.name`
           - `joint_positions` by zipping `message.name` with `message.position`
           - `gripper_position` from the first joint whose name contains `gripper`, if present
           - `gripper_open_fraction` as `null` when unavailable
           - `tool_pose` as `null` when unavailable
        5. Only return `STATE_UNAVAILABLE` after checking the manifest and trying the best available state topic.
        """
    }

    nonisolated static var namedPoseDiscoveryGuidance: String {
        """
        Resolve named poses in this order:
        1. Prefer `/follower/list_named_poses` or any service whose final path component is `list_named_poses`.
        2. If the service is available, call it and use the returned pose names plus workspace bounds.
        3. Otherwise prefer `/follower/named_pose_catalog` or any topic whose final path component is `named_pose_catalog`, type `std_msgs/msg/String`.
        4. If the catalog topic is available, use `ros2_subscribe_once`, parse the JSON string from `message.data`, and extract `pose_names` plus `workspace_bounds`.
        """
    }

    nonisolated static func gripperCommandGuidance(open: Bool) -> String {
        let action = open ? "open" : "close"
        let normalizedValue = open ? "1.0" : "0.0"
        return """
        Resolve the \(action)-gripper interface in this order:
        1. Prefer `/follower/\(action)_gripper` or any service whose final path component is `\(action)_gripper`, type `std_srvs/srv/Trigger`.
        2. Otherwise prefer `/follower/gripper_command` or any topic whose final path component is `gripper_command`, type `std_msgs/msg/Float64`, and publish `{data: \(normalizedValue)}`.
        3. If the robot command succeeds but state refresh fails, still return `ok:true` with `state:null` and mention that state is unavailable in `summary`.
        """
    }

    nonisolated static var cartesianCommandGuidance: String {
        """
        Resolve the Cartesian goal interface in this order:
        1. Before moving, fetch current arm odometry/state using `\(stateResolutionGuidance)`.
        2. Read `tool_pose` and `workspace_bounds` from that state when present.
        3. Prefer `/follower/convert_cartesian_coordinates` or any service whose final path component is `convert_cartesian_coordinates`, type `so101_pose_registry_interfaces/srv/ConvertCartesianCoordinates`, when the request is supplied in centimeters.
        4. If the conversion service is available, call it with `{frame_id: <request frame>, x_cm: <float>, y_cm: <float>, z_cm: <float>}` and use the returned `point` as the PoseStamped position source.
        5. Then prefer `/follower/move_to_cartesian_goal` or any service whose final path component is `move_to_cartesian_goal`, type `so101_pose_registry_interfaces/srv/MoveToCartesianGoal`.
        6. If that move service is available, call it with `{pose: <requested PoseStamped>, duration_sec: 3.0}` and use the returned `result.reason`, `result.accepted`, and `result.resolved_pose`.
        7. Otherwise prefer `/follower/cartesian_goal` or any topic whose final path component is `cartesian_goal`, type `geometry_msgs/msg/PoseStamped`.
        8. Publish the requested pose to the discovered topic.
        9. If the publish or service call succeeds but state refresh fails, still return `ok:true` with `state:null` and mention that state is unavailable in `summary`.
        """
    }

    nonisolated static func namedPoseCommandGuidance(name: String) -> String {
        """
        Resolve the named-pose interface in this order:
        1. Prefer `/follower/move_to_named_pose` or any service whose final path component is `move_to_named_pose`, type `so101_pose_registry_interfaces/srv/MoveToNamedPose`.
        2. Call the discovered service with `{name: "\(name)", duration_sec: 3.0}`.
        3. Before calling the move service, use `\(namedPoseDiscoveryGuidance)` to confirm that `\(name)` exists.
        4. If the named-pose move succeeds but state refresh fails, still return `ok:true` with `state:null` and mention that state is unavailable in `summary`.
        """
    }

    nonisolated static var estopCommandGuidance: String {
        """
        Resolve the emergency-stop interface in this order:
        1. Prefer `/rosclaw/estop` or any topic whose final path component is `estop`, type `std_msgs/msg/Bool`, and publish `{data: true}`.
        2. Otherwise prefer any service whose final path component contains `estop` or `stop` and invoke it with the discovered service type.
        3. Otherwise, if the manifest exposes an arm joint-trajectory command path, publish an empty `trajectory_msgs/msg/JointTrajectory` to halt active arm motion.
        4. If the stop command succeeds but state refresh fails, still return `ok:true` with `state:null` and mention that state is unavailable in `summary`.
        """
    }
}

struct GatewayRosclawStatePayload: Decodable, Sendable {
    let state: GatewayFollowerState?
    let estopLatched: Bool
    let lastCommandID: String?
    let updatedAt: String?

    enum CodingKeys: String, CodingKey {
        case state
        case estop
        case estopLatched = "estop_latched"
        case estopLatchedCamel = "estopLatched"
        case lastCommandID = "last_command_id"
        case lastCommandIDCamel = "lastCommandId"
        case updatedAt = "updated_at"
        case updatedAtCamel = "updatedAt"
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        if container.contains(.state) {
            self.state = try container.decodeIfPresent(GatewayFollowerState.self, forKey: .state)
        } else {
            self.state = try? GatewayFollowerState(from: decoder)
        }
        self.estopLatched =
            GatewayRosclawStatePayload.decodeEstopLatched(from: container) ?? false
        let snakeLastCommandID = try container.decodeIfPresent(String.self, forKey: .lastCommandID)
        let camelLastCommandID = try container.decodeIfPresent(String.self, forKey: .lastCommandIDCamel)
        self.lastCommandID = snakeLastCommandID ?? camelLastCommandID
        let snakeUpdatedAt = try container.decodeIfPresent(String.self, forKey: .updatedAt)
        let camelUpdatedAt = try container.decodeIfPresent(String.self, forKey: .updatedAtCamel)
        self.updatedAt = snakeUpdatedAt ?? camelUpdatedAt
    }

    nonisolated static func decodeEstopLatched(
        from container: KeyedDecodingContainer<CodingKeys>
    ) -> Bool? {
        if let latched = try? container.decodeIfPresent(Bool.self, forKey: .estopLatched) {
            return latched
        }
        if let latched = try? container.decodeIfPresent(Bool.self, forKey: .estopLatchedCamel) {
            return latched
        }
        if let estopObject = try? container.decodeIfPresent([String: Bool].self, forKey: .estop) {
            return estopObject["latched"] ?? estopObject["isLatched"]
        }
        if let estopBool = try? container.decodeIfPresent(Bool.self, forKey: .estop) {
            return estopBool
        }
        return nil
    }
}

private struct GatewayWorkspaceBoundsPayload: Decodable, Sendable {
    let xMin: Double
    let xMax: Double
    let yMin: Double
    let yMax: Double
    let zMin: Double
    let zMax: Double

    enum CodingKeys: String, CodingKey {
        case xMin
        case xMax
        case yMin
        case yMax
        case zMin
        case zMax
        case xMinSnake = "x_min"
        case xMaxSnake = "x_max"
        case yMinSnake = "y_min"
        case yMaxSnake = "y_max"
        case zMinSnake = "z_min"
        case zMaxSnake = "z_max"
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.xMin = try container.decodeIfPresent(Double.self, forKey: .xMin) ?? container.decode(Double.self, forKey: .xMinSnake)
        self.xMax = try container.decodeIfPresent(Double.self, forKey: .xMax) ?? container.decode(Double.self, forKey: .xMaxSnake)
        self.yMin = try container.decodeIfPresent(Double.self, forKey: .yMin) ?? container.decode(Double.self, forKey: .yMinSnake)
        self.yMax = try container.decodeIfPresent(Double.self, forKey: .yMax) ?? container.decode(Double.self, forKey: .yMaxSnake)
        self.zMin = try container.decodeIfPresent(Double.self, forKey: .zMin) ?? container.decode(Double.self, forKey: .zMinSnake)
        self.zMax = try container.decodeIfPresent(Double.self, forKey: .zMax) ?? container.decode(Double.self, forKey: .zMaxSnake)
    }

    nonisolated var workspaceBounds: WorkspaceBounds {
        WorkspaceBounds(
            xMin: xMin,
            xMax: xMax,
            yMin: yMin,
            yMax: yMax,
            zMin: zMin,
            zMax: zMax
        )
    }
}

struct GatewayRosclawBootstrapPayload: Decodable, Sendable {
    let executorAgentID: String?
    let executorSessionKey: String?
    let supportedCommands: [String]
    let presets: [String]
    let workspaceBounds: WorkspaceBounds?
    let requiredNodeCommands: [String]
    let statePayload: GatewayRosclawStatePayload?
    let estopLatchedValue: Bool?

    enum CodingKeys: String, CodingKey {
        case executorAgentID = "executorAgentId"
        case executorAgentIDSnake = "executor_agent_id"
        case executorSessionKey
        case executorSessionKeySnake = "executor_session_key"
        case supportedCommands
        case supportedCommandsSnake = "supported_commands"
        case presets
        case workspaceBounds
        case workspaceBoundsSnake = "workspace_bounds"
        case requiredNodeCommands
        case requiredNodeCommandsSnake = "required_node_commands"
        case estop
        case estopLatched = "estop_latched"
        case estopLatchedCamel = "estopLatched"
        case state
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        let camelExecutorAgentID = try container.decodeIfPresent(String.self, forKey: .executorAgentID)
        let snakeExecutorAgentID = try container.decodeIfPresent(String.self, forKey: .executorAgentIDSnake)
        self.executorAgentID = camelExecutorAgentID ?? snakeExecutorAgentID
        let camelExecutorSessionKey = try container.decodeIfPresent(String.self, forKey: .executorSessionKey)
        let snakeExecutorSessionKey = try container.decodeIfPresent(String.self, forKey: .executorSessionKeySnake)
        self.executorSessionKey = camelExecutorSessionKey ?? snakeExecutorSessionKey
        let camelSupportedCommands = try container.decodeIfPresent([String].self, forKey: .supportedCommands)
        let snakeSupportedCommands = try container.decodeIfPresent([String].self, forKey: .supportedCommandsSnake)
        self.supportedCommands = camelSupportedCommands ?? snakeSupportedCommands ?? []
        self.presets = try container.decodeIfPresent([String].self, forKey: .presets) ?? []
        let camelWorkspaceBounds = try container.decodeIfPresent(GatewayWorkspaceBoundsPayload.self, forKey: .workspaceBounds)
        let snakeWorkspaceBounds = try container.decodeIfPresent(GatewayWorkspaceBoundsPayload.self, forKey: .workspaceBoundsSnake)
        let workspaceBoundsPayload = camelWorkspaceBounds ?? snakeWorkspaceBounds
        self.workspaceBounds = workspaceBoundsPayload?.workspaceBounds
        let camelRequiredNodeCommands = try container.decodeIfPresent([String].self, forKey: .requiredNodeCommands)
        let snakeRequiredNodeCommands = try container.decodeIfPresent([String].self, forKey: .requiredNodeCommandsSnake)
        self.requiredNodeCommands = camelRequiredNodeCommands ?? snakeRequiredNodeCommands ?? []
        self.estopLatchedValue = Self.decodeEstopLatched(from: container)
        self.statePayload = try container.decodeIfPresent(GatewayRosclawStatePayload.self, forKey: .state)
    }

    nonisolated var capabilities: GatewayRobotCapabilities {
        let fallbackPresets = statePayload?.state?.namedPoseNames ?? []
        return GatewayRobotCapabilities.fromOracle(
            supportedCommands: supportedCommands,
            supportedPresets: presets.isEmpty ? fallbackPresets : presets,
            workspaceBounds: workspaceBounds ?? statePayload?.state?.workspaceBounds
        )
    }

    nonisolated var estopLatched: Bool {
        statePayload?.estopLatched ?? estopLatchedValue ?? false
    }

    nonisolated private static func decodeEstopLatched(
        from container: KeyedDecodingContainer<CodingKeys>
    ) -> Bool? {
        if let latched = try? container.decodeIfPresent(Bool.self, forKey: .estopLatched) {
            return latched
        }
        if let latched = try? container.decodeIfPresent(Bool.self, forKey: .estopLatchedCamel) {
            return latched
        }
        if let estopObject = try? container.decodeIfPresent([String: Bool].self, forKey: .estop) {
            return estopObject["latched"] ?? estopObject["isLatched"]
        }
        if let estopBool = try? container.decodeIfPresent(Bool.self, forKey: .estop) {
            return estopBool
        }
        return nil
    }
}

enum GatewayRobotExecutorError: LocalizedError {
    case invalidConfiguration(String)
    case noAgentsAvailable
    case malformedResponse(String)
    case remoteFailure(String)
    case unsupportedCommand(String)

    var errorDescription: String? {
        switch self {
        case .invalidConfiguration(let message),
             .malformedResponse(let message),
             .remoteFailure(let message),
             .unsupportedCommand(let message):
            return message
        case .noAgentsAvailable:
            return "The gateway did not return any runnable agents."
        }
    }
}

private struct GatewayAgentsListResponse: Decodable {
    struct AgentRow: Decodable {
        let id: String
        let name: String?

        nonisolated init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            self.id = try container.decode(String.self, forKey: .id)
            self.name = try container.decodeIfPresent(String.self, forKey: .name)
        }

        private enum CodingKeys: String, CodingKey {
            case id
            case name
        }
    }

    let defaultId: String
    let agents: [AgentRow]

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.defaultId = try container.decode(String.self, forKey: .defaultId)
        self.agents = try container.decode([AgentRow].self, forKey: .agents)
    }

    private enum CodingKeys: String, CodingKey {
        case defaultId
        case agents
    }
}

private struct GatewayAgentAcceptedResponse: Decodable {
    let runId: String
    let status: String

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.runId = try container.decode(String.self, forKey: .runId)
        self.status = try container.decode(String.self, forKey: .status)
    }

    private enum CodingKeys: String, CodingKey {
        case runId
        case status
    }
}

private struct GatewayAgentWaitResponse: Decodable {
    let runId: String
    let status: String
    let error: String?

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.runId = try container.decode(String.self, forKey: .runId)
        self.status = try container.decode(String.self, forKey: .status)
        self.error = try container.decodeIfPresent(String.self, forKey: .error)
    }

    private enum CodingKeys: String, CodingKey {
        case runId
        case status
        case error
    }
}

private struct GatewayChatHistoryResponse: Decodable {
    let sessionKey: String
    let messages: [GatewayChatMessage]?

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.sessionKey = try container.decode(String.self, forKey: .sessionKey)
        self.messages = try container.decodeIfPresent([GatewayChatMessage].self, forKey: .messages)
    }

    private enum CodingKeys: String, CodingKey {
        case sessionKey
        case messages
    }
}

private struct GatewayChatMessage: Decodable {
    let role: String
    let content: [GatewayChatMessageContent]

    enum CodingKeys: String, CodingKey {
        case role
        case content
    }

    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.role = try container.decode(String.self, forKey: .role)

        if let items = try? container.decode([GatewayChatMessageContent].self, forKey: .content) {
            self.content = items
            return
        }

        if let text = try? container.decode(String.self, forKey: .content) {
            self.content = [GatewayChatMessageContent(type: "text", text: text, content: nil)]
            return
        }

        self.content = []
    }
}

private struct GatewayChatMessageContent: Decodable {
    let type: String?
    let text: String?
    let content: AnyCodable?

    nonisolated init(type: String?, text: String?, content: AnyCodable?) {
        self.type = type
        self.text = text
        self.content = content
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.type = try container.decodeIfPresent(String.self, forKey: .type)
        self.text = try container.decodeIfPresent(String.self, forKey: .text)
        self.content = try container.decodeIfPresent(AnyCodable.self, forKey: .content)
    }

    private enum CodingKeys: String, CodingKey {
        case type
        case text
        case content
    }
}

private struct GatewaySnapshotFetchResult: Sendable {
    let state: GatewayFollowerState?
    let summary: String?

    nonisolated init(state: GatewayFollowerState?, summary: String?) {
        self.state = state
        self.summary = summary?.trimmedNonEmpty
    }
}

private struct GatewayRemoteEnvelope: Decodable {
    let schemaVersion: String?
    let kind: String?
    let intentId: String?
    let ok: Bool?
    let summary: String?
    let errorCode: String?
    let error: String?
    let estopLatched: Bool?
    let state: GatewayFollowerState?

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.schemaVersion = try container.decodeIfPresent(String.self, forKey: .schemaVersion)
        self.kind = try container.decodeIfPresent(String.self, forKey: .kind)
        self.intentId = try container.decodeIfPresent(String.self, forKey: .intentId)
        self.ok = try container.decodeIfPresent(Bool.self, forKey: .ok)
        self.summary = try container.decodeIfPresent(String.self, forKey: .summary)
        self.errorCode = try container.decodeIfPresent(String.self, forKey: .errorCode)
        self.error = try container.decodeIfPresent(String.self, forKey: .error)
        self.estopLatched = try container.decodeIfPresent(Bool.self, forKey: .estopLatched)
        self.state = try container.decodeIfPresent(GatewayFollowerState.self, forKey: .state)
    }

    private enum CodingKeys: String, CodingKey {
        case schemaVersion
        case kind
        case intentId
        case ok
        case summary
        case errorCode
        case error
        case estopLatched
        case state
    }
}

struct GatewayPendingDevicePairingRequest: Decodable, Sendable {
    let requestId: String
    let deviceId: String
    let displayName: String?
    let clientId: String?
    let clientMode: String?
    let role: String?
    let roles: [String]?
    let scopes: [String]?
    let ts: Int?

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.requestId = try container.decode(String.self, forKey: .requestId)
        self.deviceId = try container.decode(String.self, forKey: .deviceId)
        self.displayName = try container.decodeIfPresent(String.self, forKey: .displayName)
        self.clientId = try container.decodeIfPresent(String.self, forKey: .clientId)
        self.clientMode = try container.decodeIfPresent(String.self, forKey: .clientMode)
        self.role = try container.decodeIfPresent(String.self, forKey: .role)
        self.roles = try container.decodeIfPresent([String].self, forKey: .roles)
        self.scopes = try container.decodeIfPresent([String].self, forKey: .scopes)
        self.ts = try container.decodeIfPresent(Int.self, forKey: .ts)
    }

    private enum CodingKeys: String, CodingKey {
        case requestId
        case deviceId
        case displayName
        case clientId
        case clientMode
        case role
        case roles
        case scopes
        case ts
    }
}

private struct GatewayDevicePairingListResponse: Decodable {
    let pending: [GatewayPendingDevicePairingRequest]

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.pending = try container.decodeIfPresent([GatewayPendingDevicePairingRequest].self, forKey: .pending) ?? []
    }

    private enum CodingKeys: String, CodingKey {
        case pending
    }
}

actor GatewayRobotExecutor {
    private let operatorGateway = GatewayNodeSession()
    private let nodeGateway = GatewayNodeSession()
    private let encoder = JSONEncoder()
    private let decoder = JSONDecoder()

    private var endpoint: GatewayRobotEndpointConfiguration?
    private var executorAgentID: String?
    private var executorSessionKey: String = "visionos-robot"
    private var nodeID: String = "openclaw-ios"
    private var capabilities: GatewayRobotCapabilities = .followerDefault
    private var latestEstopLatched = false
    private var latestStateSummary: String?
    private var typedGatewayMethodsAvailable = true
    private var snapshotUpdateHandler: (@Sendable (GatewayRobotSnapshot) async -> Void)?

    func disconnect() async {
        await self.operatorGateway.disconnect()
        await self.nodeGateway.disconnect()
        self.endpoint = nil
        self.executorAgentID = nil
        self.capabilities = .followerDefault
        self.latestEstopLatched = false
        self.latestStateSummary = nil
        self.typedGatewayMethodsAvailable = true
        self.snapshotUpdateHandler = nil
    }

    func bootstrap(
        configuration: GatewayRobotEndpointConfiguration,
        snapshotUpdateHandler: (@Sendable (GatewayRobotSnapshot) async -> Void)? = nil,
        nodeInvokeHandler: @escaping @Sendable (BridgeInvokeRequest) async -> BridgeInvokeResponse
    ) async throws -> GatewayRobotSnapshot {
        self.endpoint = configuration
        self.executorSessionKey = configuration.executorSessionKey
        self.snapshotUpdateHandler = snapshotUpdateHandler
        self.latestStateSummary = nil

        let baseOperatorOptions = GatewayConnectOptions(
            role: "operator",
            scopes: GatewayOperatorConnectScopes.oracle,
            caps: ["tool-events"],
            commands: [],
            permissions: [:],
            clientId: configuration.operatorClientID,
            clientMode: "ui",
            clientDisplayName: configuration.displayName,
            includeDeviceIdentity: true
        )

        let operatorOptions = try await self.connectWithClientIDFallback(
            session: self.operatorGateway,
            configuration: configuration,
            options: baseOperatorOptions,
            onEvent: { [self] event, payload in
                await self.handleOperatorEvent(event: event, payload: payload)
            },
            onInvoke: { request in
                BridgeInvokeResponse(
                    id: request.id,
                    ok: false,
                    error: OpenClawNodeError(
                        code: .invalidRequest,
                        message: "operator session cannot handle node.invoke"))
            }
        )

        let preferredNodeClientID =
            configuration.nodeClientID == configuration.operatorClientID
            ? operatorOptions.clientId
            : configuration.nodeClientID
        let baseNodeOptions = GatewayConnectOptions(
            role: "node",
            scopes: [],
            caps: [],
            commands: ["vision.calibration.get", "vision.goal.confirm", "vision.anchor.snapshot"],
            permissions: [:],
            clientId: preferredNodeClientID,
            clientMode: "node",
            clientDisplayName: configuration.displayName,
            includeDeviceIdentity: true
        )

        let nodeOptions = try await self.connectNodeWithAutomaticSelfApproval(
            configuration: configuration,
            options: baseNodeOptions,
            onInvoke: nodeInvokeHandler
        )
        self.nodeID = GatewayRuntimeNodeIdentity.resolve(
            includeDeviceIdentity: nodeOptions.includeDeviceIdentity,
            clientID: nodeOptions.clientId,
            deviceID: nodeOptions.includeDeviceIdentity ? GatewayDeviceIdentityStore.loadOrCreate().deviceId : nil
        )

        let typedBootstrap = try await self.fetchTypedBootstrapIfAvailable()
        if let typedBootstrap {
            self.capabilities = typedBootstrap.capabilities
            self.latestEstopLatched = typedBootstrap.estopLatched
            self.latestStateSummary =
                typedBootstrap.statePayload?.state == nil
                ? Self.defaultStateUnavailableSummary
                : nil
            if let typedSessionKey = typedBootstrap.executorSessionKey?.trimmedNonEmpty {
                self.executorSessionKey = typedSessionKey
            }
            if let typedAgentID = typedBootstrap.executorAgentID?.trimmedNonEmpty {
                self.executorAgentID = typedAgentID
            }
        } else {
            self.capabilities = .followerDefault
            self.latestEstopLatched = false
        }

        if self.executorAgentID == nil {
            let agents = try await self.listAgents()
            let selectedAgentID = self.resolveExecutorAgentID(
                from: agents,
                hint: configuration.executorAgentIDHint
            )
            self.executorAgentID = selectedAgentID
        }

        guard let executorAgentID = self.executorAgentID else {
            throw GatewayRobotExecutorError.noAgentsAvailable
        }

        let snapshotFetchResult: GatewaySnapshotFetchResult
        if let bootstrapState = typedBootstrap?.statePayload?.state {
            snapshotFetchResult = GatewaySnapshotFetchResult(state: bootstrapState, summary: nil)
        } else {
            snapshotFetchResult = try await self.fetchSnapshotState()
        }
        self.latestStateSummary = snapshotFetchResult.summary
        return self.makeSnapshot(executorAgentID: executorAgentID, state: snapshotFetchResult.state)
    }

    private func connectWithClientIDFallback(
        session: GatewayNodeSession,
        configuration: GatewayRobotEndpointConfiguration,
        options: GatewayConnectOptions,
        onEvent: (@Sendable (String, AnyCodable?) async -> Void)?,
        onInvoke: @escaping @Sendable (BridgeInvokeRequest) async -> BridgeInvokeResponse
    ) async throws -> GatewayConnectOptions {
        var effectiveOptions = options
        var retriedLegacyClientID = false

        while true {
            do {
                try await session.connect(
                    url: configuration.gatewayURL,
                    token: configuration.authToken,
                    bootstrapToken: configuration.bootstrapToken,
                    password: configuration.password,
                    connectOptions: effectiveOptions,
                    sessionBox: nil,
                    onConnected: {},
                    onDisconnected: { _ in },
                    onEvent: onEvent,
                    onInvoke: onInvoke
                )
                return effectiveOptions
            } catch {
                guard !retriedLegacyClientID,
                      let fallbackClientID = GatewayVisionOSClientID.legacyFallback(
                          currentClientID: effectiveOptions.clientId,
                          error: error)
                else {
                    throw error
                }
                retriedLegacyClientID = true
                effectiveOptions = effectiveOptions.replacing(clientID: fallbackClientID)
            }
        }
    }

    func refreshSnapshot() async throws -> GatewayRobotSnapshot {
        guard let executorAgentID else {
            throw GatewayRobotExecutorError.invalidConfiguration("Gateway executor has not been bootstrapped.")
        }

        let result = try await self.fetchSnapshotState()
        self.latestStateSummary = result.summary
        return self.makeSnapshot(executorAgentID: executorAgentID, state: result.state)
    }

    func execute(
        command: RobotCommand,
        workspaceSnapshot: GatewayWorkspaceSnapshot?
    ) async throws -> GatewayRobotCommandResult {
        let intentID = UUID().uuidString
        let prompt = try self.prompt(for: command, intentID: intentID, workspaceSnapshot: workspaceSnapshot)
        return try await self.runCommandPrompt(prompt, intentID: intentID)
    }

    func requestEstop(reason: String, workspaceSnapshot: GatewayWorkspaceSnapshot?) async throws -> GatewayRobotCommandResult {
        let intentID = UUID().uuidString
        let prompt = """
        You are the ROSClaw executor for an SO-101 follower arm.

        Execute an immediate software emergency stop for intent "\(intentID)".

        \(GatewayRosclawPromptOracle.manifestResolutionGuidance)

        \(GatewayRosclawPromptOracle.estopCommandGuidance)

        After the stop attempt, refresh robot state using these rules:
        \(GatewayRosclawPromptOracle.stateResolutionGuidance)

        Additional context:
        - reason: \(reason)
        - headsetNodeId: \(self.nodeID)
        - workspaceSnapshot:
        \(workspaceSnapshot?.promptSnippet ?? "- unavailable")

        Return ONLY valid JSON with this shape:
        {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"short summary","errorCode":null,"estopLatched":true,"state":{"joint_order":[],"joint_positions":{},"gripper_position":null,"gripper_open_fraction":null,"tool_pose":null}}

        If the stop succeeds but state cannot be refreshed, return:
        {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"software emergency stop sent; state unavailable","errorCode":null,"estopLatched":true,"state":null}

        If the stop cannot be performed, return:
        {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":false,"summary":"failure summary","errorCode":"ESTOP_FAILED","estopLatched":true,"state":null}

        Respond with JSON only. No markdown.
        """

        return try await self.runCommandPrompt(prompt, intentID: intentID)
    }

    private func makeSnapshot(
        executorAgentID: String,
        state: GatewayFollowerState?
    ) -> GatewayRobotSnapshot {
        let capabilities = self.capabilities.enriching(with: state)
        self.capabilities = capabilities
        return GatewayRobotSnapshot(
            executorAgentID: executorAgentID,
            executorSessionKey: self.executorSessionKey,
            nodeID: self.nodeID,
            capabilities: capabilities,
            state: state,
            stateSummary: self.latestStateSummary,
            estopLatched: self.latestEstopLatched
        )
    }

    private func handleOperatorEvent(event: String, payload: AnyCodable?) async {
        guard event == "rosclaw.state",
              let payload,
              let decoded = try? self.decodeRosclawStatePayload(from: payload),
              let executorAgentID = self.executorAgentID
        else {
            return
        }

        self.latestEstopLatched = decoded.estopLatched
        self.latestStateSummary = decoded.state == nil ? Self.defaultStateUnavailableSummary : nil
        guard let snapshotUpdateHandler else {
            return
        }
        await snapshotUpdateHandler(self.makeSnapshot(executorAgentID: executorAgentID, state: decoded.state))
    }

    private func fetchTypedBootstrap() async throws -> GatewayRosclawBootstrapPayload {
        let response = try await self.operatorGateway.request(
            method: "rosclaw.bootstrap",
            paramsJSON: "{}",
            timeoutSeconds: 15
        )
        return try self.decoder.decode(GatewayRosclawBootstrapPayload.self, from: response)
    }

    private func listAgents() async throws -> GatewayAgentsListResponse {
        let response = try await self.operatorGateway.request(method: "agents.list", paramsJSON: "{}", timeoutSeconds: 15)
        return try self.decoder.decode(GatewayAgentsListResponse.self, from: response)
    }

    private func resolveExecutorAgentID(
        from response: GatewayAgentsListResponse,
        hint: String?
    ) -> String {
        if let hint {
            if let match = response.agents.first(where: { $0.id.caseInsensitiveCompare(hint) == .orderedSame }) {
                return match.id
            }
        }

        let preferred = response.agents.first {
            let haystack = "\($0.id) \($0.name ?? "")".lowercased()
            return haystack.contains("rosclaw") || haystack.contains("robot") || haystack.contains("so101")
        }
        if let preferred {
            return preferred.id
        }

        if let defaultMatch = response.agents.first(where: { $0.id == response.defaultId }) {
            return defaultMatch.id
        }

        if let first = response.agents.first {
            return first.id
        }

        return response.defaultId
    }

    private func fetchSnapshotState() async throws -> GatewaySnapshotFetchResult {
        if self.typedGatewayMethodsAvailable {
            do {
                let typedState = try await self.fetchTypedState()
                self.latestEstopLatched = typedState.estopLatched
                if let typedStatePayload = typedState.state {
                    return GatewaySnapshotFetchResult(state: typedStatePayload, summary: nil)
                }
                return GatewaySnapshotFetchResult(state: nil, summary: Self.defaultStateUnavailableSummary)
            } catch {
                if Self.shouldDisableTypedGatewayMethods(for: error) {
                    self.typedGatewayMethodsAvailable = false
                }
            }
        }

        let prompt = """
        You are the ROSClaw executor for an SO-101 follower arm.

        Read the current robot state and return ONLY valid JSON.

        \(GatewayRosclawPromptOracle.manifestResolutionGuidance)

        Procedure:
        \(GatewayRosclawPromptOracle.stateResolutionGuidance)

        Return ONLY one of these JSON objects:
        {"schemaVersion":"1","kind":"snapshot","ok":true,"summary":"state synchronized","state":{"joint_order":[],"joint_positions":{},"gripper_position":null,"gripper_open_fraction":null,"tool_pose":null}}
        {"schemaVersion":"1","kind":"snapshot","ok":false,"summary":"state unavailable","errorCode":"STATE_UNAVAILABLE","state":null}

        No markdown. No explanation.
        """

        let envelope = try await self.runPrompt(prompt)
        guard envelope.kind == "snapshot" || envelope.kind == nil else {
            throw GatewayRobotExecutorError.malformedResponse("Unexpected snapshot envelope kind: \(envelope.kind ?? "nil")")
        }
        if envelope.ok == false {
            if Self.isStateUnavailableEnvelope(envelope) {
                return GatewaySnapshotFetchResult(
                    state: nil,
                    summary: Self.stateUnavailableSummary(from: envelope.summary)
                )
            }
            throw GatewayRobotExecutorError.remoteFailure(envelope.summary ?? envelope.error ?? "Robot state is unavailable.")
        }
        if let state = envelope.state {
            return GatewaySnapshotFetchResult(state: state, summary: nil)
        }
        return GatewaySnapshotFetchResult(
            state: nil,
            summary: Self.stateUnavailableSummary(from: envelope.summary)
        )
    }

    private func fetchTypedState() async throws -> GatewayRosclawStatePayload {
        let response = try await self.operatorGateway.request(
            method: "rosclaw.state.get",
            paramsJSON: "{}",
            timeoutSeconds: 15
        )
        return try self.decoder.decode(GatewayRosclawStatePayload.self, from: response)
    }

    private func connectNodeWithAutomaticSelfApproval(
        configuration: GatewayRobotEndpointConfiguration,
        options: GatewayConnectOptions,
        onInvoke: @escaping @Sendable (BridgeInvokeRequest) async -> BridgeInvokeResponse
    ) async throws -> GatewayConnectOptions {
        do {
            return try await self.connectWithClientIDFallback(
                session: self.nodeGateway,
                configuration: configuration,
                options: options,
                onEvent: nil,
                onInvoke: onInvoke
            )
        } catch {
            guard Self.isPairingRequired(error),
                  await self.tryAutoApproveOwnNodePairing(displayName: configuration.displayName)
            else {
                throw error
            }
        }

        return try await self.connectWithClientIDFallback(
            session: self.nodeGateway,
            configuration: configuration,
            options: options,
            onEvent: nil,
            onInvoke: onInvoke
        )
    }

    private func tryAutoApproveOwnNodePairing(displayName: String?) async -> Bool {
        do {
            let deviceID = GatewayDeviceIdentityStore.loadOrCreate().deviceId
            let pairingList = try await self.listDevicePairings()
            guard let candidate = Self.selectPendingOwnNodePairing(
                from: pairingList.pending,
                deviceID: deviceID,
                displayName: displayName
            ) else {
                return false
            }

            struct Params: Encodable {
                let requestId: String
            }

            _ = try await self.operatorGateway.request(
                method: "device.pair.approve",
                paramsJSON: try self.encodeJSONString(Params(requestId: candidate.requestId)),
                timeoutSeconds: 15
            )
            try? await Task.sleep(nanoseconds: 250_000_000)
            return true
        } catch {
            return false
        }
    }

    private func listDevicePairings() async throws -> GatewayDevicePairingListResponse {
        let response = try await self.operatorGateway.request(
            method: "device.pair.list",
            paramsJSON: "{}",
            timeoutSeconds: 15
        )
        return try self.decoder.decode(GatewayDevicePairingListResponse.self, from: response)
    }

    private func prompt(
        for command: RobotCommand,
        intentID: String,
        workspaceSnapshot: GatewayWorkspaceSnapshot?
    ) throws -> String {
        let workspaceSnippet = workspaceSnapshot?.promptSnippet ?? "- unavailable"

        switch command {
        case .openGripper:
            return """
            You are the ROSClaw executor for an SO-101 follower arm.

            Execute intent "\(intentID)":
            - command: open_gripper
            - headsetNodeId: \(self.nodeID)
            - workspaceSnapshot:
            \(workspaceSnippet)

            \(GatewayRosclawPromptOracle.manifestResolutionGuidance)

            Procedure:
            \(GatewayRosclawPromptOracle.gripperCommandGuidance(open: true))

            After the command attempt, refresh robot state using these rules:
            \(GatewayRosclawPromptOracle.stateResolutionGuidance)

            Return ONLY valid JSON:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"gripper opened","errorCode":null,"estopLatched":false,"state":{"joint_order":[],"joint_positions":{},"gripper_position":null,"gripper_open_fraction":null,"tool_pose":null}}

            If the command succeeds but state cannot be refreshed, return:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"gripper opened; state unavailable","errorCode":null,"estopLatched":false,"state":null}

            If the command fails, return:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":false,"summary":"gripper open failed","errorCode":"OPEN_GRIPPER_FAILED","estopLatched":false,"state":null}

            Respond with JSON only. No markdown.
            """

        case .closeGripper:
            return """
            You are the ROSClaw executor for an SO-101 follower arm.

            Execute intent "\(intentID)":
            - command: close_gripper
            - headsetNodeId: \(self.nodeID)
            - workspaceSnapshot:
            \(workspaceSnippet)

            \(GatewayRosclawPromptOracle.manifestResolutionGuidance)

            Procedure:
            \(GatewayRosclawPromptOracle.gripperCommandGuidance(open: false))

            After the command attempt, refresh robot state using these rules:
            \(GatewayRosclawPromptOracle.stateResolutionGuidance)

            Return ONLY valid JSON:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"gripper closed","errorCode":null,"estopLatched":false,"state":{"joint_order":[],"joint_positions":{},"gripper_position":null,"gripper_open_fraction":null,"tool_pose":null}}

            If the command succeeds but state cannot be refreshed, return:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"gripper closed; state unavailable","errorCode":null,"estopLatched":false,"state":null}

            If the command fails, return:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":false,"summary":"gripper close failed","errorCode":"CLOSE_GRIPPER_FAILED","estopLatched":false,"state":null}

            Respond with JSON only. No markdown.
            """

        case .moveToCartesian(let target):
            let requestedMeters = target.requestedMeters
            let requestedCentimeters = target.requestedCentimeters
            let frameID = target.referenceFrame == .toolRelative ? FollowerTopics.toolFrame : FollowerTopics.baseFrame
            let resolvedMeterSummary = String(
                format: "(%.3f, %.3f, %.3f) m",
                requestedMeters.x,
                requestedMeters.y,
                requestedMeters.z
            )
            let successSummaryExample =
                "requested \(target.requestSummary) [\(target.frameDisplayName)] → resolved \(resolvedMeterSummary)"
            let stateUnavailableSummaryExample =
                "requested \(target.requestSummary) [\(target.frameDisplayName)]; state unavailable"
            let orientationInstruction: String
            switch target.referenceFrame {
            case .baseAbsolute:
                orientationInstruction = """
                8. Use identity orientation for the request pose:
                   {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                   The backend owns the final safe tool orientation for base-frame requests, so do not invent a frontend-specific safe quaternion.
                """
            case .toolRelative:
                orientationInstruction = """
                8. Use identity orientation for the request pose:
                   {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                   In the gripper frame this preserves the current tool-relative orientation, as documented by the backend oracle.
                """
            }
            let requestSnippet: String
            let procedureSnippet: String
            if target.source == .voice {
                requestSnippet = """
                - requestedPositionUser: {"x": \(Self.format(target.x)), "y": \(Self.format(target.y)), "z": \(Self.format(target.z)), "unit": "\(target.unit.symbol)", "referenceFrame": "\(target.referenceFrame.rawValue)"}
                - requestedPositionCentimeters: {"x": \(Self.format(requestedCentimeters.x)), "y": \(Self.format(requestedCentimeters.y)), "z": \(Self.format(requestedCentimeters.z))}
                """
                procedureSnippet = """
                7. Use `\(frameID)` as the request frame.
                \(orientationInstruction)
                9. Because this command came from a spoken metric request, fetch current arm odometry/state first.
                10. If the request is tool-relative and `tool_pose` or `workspace_bounds` is unavailable, fail with `ODOMETRY_UNAVAILABLE`.
                11. Call `convert_cartesian_coordinates` with the centimeter values and use the returned `point` as the PoseStamped position.
                12. Call `move_to_cartesian_goal` with that PoseStamped.
                13. If the move service returns a resolved pose, mention both the requested \(target.requestSummary) and the resolved pose in meters in `summary`.
                """
            } else {
                requestSnippet = """
                - requestedPositionMeters: {"x": \(Self.format(requestedMeters.x)), "y": \(Self.format(requestedMeters.y)), "z": \(Self.format(requestedMeters.z)), "referenceFrame": "\(target.referenceFrame.rawValue)"}
                """
                procedureSnippet = """
                7. Use `\(frameID)` as the request frame.
                \(orientationInstruction)
                9. This request is already expressed in meters, so do not use `convert_cartesian_coordinates`.
                10. Call `move_to_cartesian_goal` with the requested pose.
                11. If the move service returns a resolved pose, mention both the requested \(target.requestSummary) and the resolved pose in meters in `summary`.
                """
            }
            return """
            You are the ROSClaw executor for an SO-101 follower arm.

            Execute intent "\(intentID)":
            - command: move_to_cartesian
            - headsetNodeId: \(self.nodeID)
            \(requestSnippet)
            - workspaceSnapshot:
            \(workspaceSnippet)

            \(GatewayRosclawPromptOracle.manifestResolutionGuidance)

            Procedure:
            \(GatewayRosclawPromptOracle.cartesianCommandGuidance)
            \(procedureSnippet)

            After the publish attempt, refresh robot state using these rules:
            \(GatewayRosclawPromptOracle.stateResolutionGuidance)

            Return ONLY valid JSON:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"\(successSummaryExample)","errorCode":null,"estopLatched":false,"state":{"joint_order":[],"joint_positions":{},"gripper_position":null,"gripper_open_fraction":null,"tool_pose":{"frame_id":"\(frameID)","position":{"x":\(Self.format(requestedMeters.x)),"y":\(Self.format(requestedMeters.y)),"z":\(Self.format(requestedMeters.z))},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}}}

            If the command succeeds but state cannot be refreshed, return:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"\(stateUnavailableSummaryExample)","errorCode":null,"estopLatched":false,"state":null}

            If execution fails, return:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":false,"summary":"cartesian move failed","errorCode":"MOVE_TO_CARTESIAN_FAILED","estopLatched":false,"state":null}

            Respond with JSON only. No markdown.
            """

        case .homeRobot:
            return try self.prompt(
                for: .moveToPreset(name: "home"),
                intentID: intentID,
                workspaceSnapshot: workspaceSnapshot
            )

        case .moveToPreset(let name):
            return """
            You are the ROSClaw executor for an SO-101 follower arm.

            Execute intent "\(intentID)":
            - command: move_to_preset
            - presetName: \(name)
            - headsetNodeId: \(self.nodeID)
            - workspaceSnapshot:
            \(workspaceSnippet)

            \(GatewayRosclawPromptOracle.manifestResolutionGuidance)

            Procedure:
            \(GatewayRosclawPromptOracle.namedPoseCommandGuidance(name: name))

            After the move attempt, refresh robot state using these rules:
            \(GatewayRosclawPromptOracle.stateResolutionGuidance)

            Return ONLY valid JSON:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"named pose move sent","errorCode":null,"estopLatched":false,"state":{"joint_order":[],"joint_positions":{},"gripper_position":null,"gripper_open_fraction":null,"tool_pose":null}}

            If the move succeeds but state cannot be refreshed, return:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":true,"summary":"named pose move sent; state unavailable","errorCode":null,"estopLatched":false,"state":null}

            If the named pose does not exist or the move fails, return:
            {"schemaVersion":"1","kind":"command_result","intentId":"\(intentID)","ok":false,"summary":"named pose move failed","errorCode":"MOVE_TO_PRESET_FAILED","estopLatched":false,"state":null}

            Respond with JSON only. No markdown.
            """

        case .pickAndPlace:
            throw GatewayRobotExecutorError.unsupportedCommand(
                "Pick-and-place is not part of the backend-backed primitive command surface.")

        case .captureSnapshot:
            throw GatewayRobotExecutorError.unsupportedCommand(
                "Camera snapshot delegation is not part of the backend-backed primitive command surface.")

        case .emergencyStop:
            throw GatewayRobotExecutorError.unsupportedCommand(
                "Use requestEstop(reason:) for emergency stop execution.")
        }
    }

    func testingPrompt(
        for command: RobotCommand,
        intentID: String = "intent-test",
        workspaceSnapshot: GatewayWorkspaceSnapshot? = nil
    ) throws -> String {
        try self.prompt(
            for: command,
            intentID: intentID,
            workspaceSnapshot: workspaceSnapshot
        )
    }

    private func runCommandPrompt(_ prompt: String, intentID: String) async throws -> GatewayRobotCommandResult {
        let envelope = try await self.runPrompt(prompt)
        guard envelope.kind == "command_result" || envelope.kind == nil else {
            throw GatewayRobotExecutorError.malformedResponse(
                "Unexpected command envelope kind: \(envelope.kind ?? "nil")")
        }
        guard envelope.intentId == nil || envelope.intentId == intentID else {
            throw GatewayRobotExecutorError.malformedResponse(
                "Remote executor responded with a mismatched intent id.")
        }

        let ok = envelope.ok ?? false
        let summary = envelope.summary ?? envelope.error ?? (ok ? "Command completed." : "Command failed.")
        let result = GatewayRobotCommandResult(
            intentID: intentID,
            ok: ok,
            summary: summary,
            state: envelope.state,
            errorCode: envelope.errorCode,
            estopLatched: envelope.estopLatched ?? false
        )
        self.latestEstopLatched = result.estopLatched
        if result.state == nil, Self.isStateUnavailable(summary: result.summary, error: nil, errorCode: result.errorCode) {
            self.latestStateSummary = Self.stateUnavailableSummary(from: result.summary)
        } else if result.state != nil {
            self.latestStateSummary = nil
        }
        if let snapshotUpdateHandler,
           let executorAgentID = self.executorAgentID
        {
            await snapshotUpdateHandler(self.makeSnapshot(executorAgentID: executorAgentID, state: result.state))
        }
        return result
    }

    private func runPrompt(_ prompt: String) async throws -> GatewayRemoteEnvelope {
        let runID = UUID().uuidString
        let accepted = try await self.startAgentRun(message: prompt, idempotencyKey: runID)
        let wait = try await self.waitForAgentRun(runID: accepted.runId)
        guard wait.status != "timeout" else {
            throw GatewayRobotExecutorError.remoteFailure("The remote ROSClaw agent timed out.")
        }

        let history = try await self.fetchHistory()
        guard let assistantText = self.latestAssistantText(from: history.messages ?? []) else {
            throw GatewayRobotExecutorError.malformedResponse(
                "The remote ROSClaw agent did not return an assistant message.")
        }

        return try self.decodeRemoteEnvelope(from: assistantText)
    }

    private func startAgentRun(message: String, idempotencyKey: String) async throws -> GatewayAgentAcceptedResponse {
        guard let executorAgentID else {
            throw GatewayRobotExecutorError.invalidConfiguration("Gateway executor agent has not been selected.")
        }

        struct Params: Encodable {
            let message: String
            let agentId: String
            let sessionKey: String
            let thinking: String
            let deliver: Bool
            let timeout: Int
            let idempotencyKey: String
        }

        let params = Params(
            message: message,
            agentId: executorAgentID,
            sessionKey: self.executorSessionKey,
            thinking: "off",
            deliver: false,
            timeout: 45,
            idempotencyKey: idempotencyKey
        )
        let paramsJSON = try self.encodeJSONString(params)
        let response = try await self.operatorGateway.request(method: "agent", paramsJSON: paramsJSON, timeoutSeconds: 20)
        return try self.decoder.decode(GatewayAgentAcceptedResponse.self, from: response)
    }

    private func waitForAgentRun(runID: String) async throws -> GatewayAgentWaitResponse {
        struct Params: Encodable {
            let runId: String
            let timeoutMs: Int
        }

        let paramsJSON = try self.encodeJSONString(Params(runId: runID, timeoutMs: 60_000))
        let response = try await self.operatorGateway.request(method: "agent.wait", paramsJSON: paramsJSON, timeoutSeconds: 65)
        let decoded = try self.decoder.decode(GatewayAgentWaitResponse.self, from: response)
        if decoded.status == "error", let error = decoded.error {
            throw GatewayRobotExecutorError.remoteFailure(error)
        }
        return decoded
    }

    private func fetchHistory() async throws -> GatewayChatHistoryResponse {
        struct Params: Encodable {
            let sessionKey: String
        }

        let paramsJSON = try self.encodeJSONString(Params(sessionKey: self.executorSessionKey))
        let response = try await self.operatorGateway.request(method: "chat.history", paramsJSON: paramsJSON, timeoutSeconds: 20)
        return try self.decoder.decode(GatewayChatHistoryResponse.self, from: response)
    }

    private func latestAssistantText(from messages: [GatewayChatMessage]) -> String? {
        for message in messages.reversed() where message.role.lowercased() == "assistant" {
            let text = message.content.compactMap { item -> String? in
                if let text = item.text?.trimmedNonEmpty {
                    return text
                }
                if let string = item.content?.stringValue?.trimmedNonEmpty {
                    return string
                }
                return nil
            }.joined(separator: "\n")

            if let text = text.trimmedNonEmpty {
                return text
            }
        }
        return nil
    }

    private func decodeRemoteEnvelope(from rawText: String) throws -> GatewayRemoteEnvelope {
        let jsonText = try Self.extractJSONObject(from: rawText)
        guard let data = jsonText.data(using: .utf8) else {
            throw GatewayRobotExecutorError.malformedResponse("Remote JSON reply was not UTF-8.")
        }
        return try self.decoder.decode(GatewayRemoteEnvelope.self, from: data)
    }

    private func decodeRosclawStatePayload(from payload: AnyCodable) throws -> GatewayRosclawStatePayload {
        let data = try self.encoder.encode(payload)
        return try self.decoder.decode(GatewayRosclawStatePayload.self, from: data)
    }

    private func fetchTypedBootstrapIfAvailable() async throws -> GatewayRosclawBootstrapPayload? {
        guard self.typedGatewayMethodsAvailable else {
            return nil
        }
        do {
            return try await self.fetchTypedBootstrap()
        } catch {
            if Self.shouldDisableTypedGatewayMethods(for: error) {
                self.typedGatewayMethodsAvailable = false
                return nil
            }
            return nil
        }
    }

    private func encodeJSONString<T: Encodable>(_ value: T) throws -> String {
        let data = try self.encoder.encode(value)
        guard let string = String(data: data, encoding: .utf8) else {
            throw GatewayRobotExecutorError.malformedResponse("Could not encode gateway request JSON.")
        }
        return string
    }

    private static func extractJSONObject(from raw: String) throws -> String {
        let trimmed = raw.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let start = trimmed.firstIndex(of: "{"),
              let end = trimmed.lastIndex(of: "}")
        else {
            throw GatewayRobotExecutorError.malformedResponse(
                "The remote ROSClaw agent did not return a JSON object.")
        }
        return String(trimmed[start...end])
    }

    private static func format(_ value: Double) -> String {
        String(format: "%.6f", value)
    }

    nonisolated static func shouldDisableTypedGatewayMethods(for error: Error) -> Bool {
        let message = error.localizedDescription.lowercased()
        return message.contains("missing scope: operator.admin")
            || message.contains("method not found")
            || message.contains("unknown method")
    }

    nonisolated static func selectPendingOwnNodePairing(
        from pending: [GatewayPendingDevicePairingRequest],
        deviceID: String,
        displayName: String?
    ) -> GatewayPendingDevicePairingRequest? {
        pending
            .filter { request in
                guard request.deviceId == deviceID else {
                    return false
                }
                guard Self.isPureNodePairingRequest(request) else {
                    return false
                }
                if let clientMode = request.clientMode?.trimmedNonEmpty,
                   clientMode.caseInsensitiveCompare("node") != .orderedSame
                {
                    return false
                }
                if let displayName,
                   let requestDisplayName = request.displayName?.trimmedNonEmpty,
                   requestDisplayName.caseInsensitiveCompare(displayName) != .orderedSame
                {
                    return false
                }
                return true
            }
            .sorted { ($0.ts ?? 0) > ($1.ts ?? 0) }
            .first
    }

    nonisolated static func isPureNodePairingRequest(_ request: GatewayPendingDevicePairingRequest) -> Bool {
        let normalizedRoles = Set(
            ([request.role].compactMap { $0 } + (request.roles ?? []))
                .map { $0.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() }
                .filter { !$0.isEmpty }
        )
        let normalizedScopes = (request.scopes ?? [])
            .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
            .filter { !$0.isEmpty }
        return normalizedRoles == ["node"] && normalizedScopes.isEmpty
    }

    nonisolated static func isPairingRequired(_ error: Error) -> Bool {
        let message = error.localizedDescription.lowercased()
        return message.contains("pairing required")
            || message.contains("not_paired")
            || message.contains("not paired")
    }

    nonisolated static func stateUnavailableSummary(from summary: String?) -> String {
        summary?.trimmedNonEmpty ?? defaultStateUnavailableSummary
    }

    nonisolated static var defaultStateUnavailableSummary: String {
        "Robot state unavailable."
    }

    nonisolated private static func isStateUnavailableEnvelope(_ envelope: GatewayRemoteEnvelope) -> Bool {
        Self.isStateUnavailable(summary: envelope.summary, error: envelope.error, errorCode: envelope.errorCode)
    }

    nonisolated private static func isStateUnavailable(
        summary: String?,
        error: String?,
        errorCode: String?
    ) -> Bool {
        if errorCode?.caseInsensitiveCompare("STATE_UNAVAILABLE") == .orderedSame {
            return true
        }
        let message = "\(summary ?? "") \(error ?? "")".lowercased()
        return message.contains("state unavailable")
    }
}

private extension GatewayConnectOptions {
    nonisolated func replacing(clientID: String) -> GatewayConnectOptions {
        GatewayConnectOptions(
            role: role,
            scopes: scopes,
            caps: caps,
            commands: commands,
            permissions: permissions,
            clientId: clientID,
            clientMode: clientMode,
            clientDisplayName: clientDisplayName,
            includeDeviceIdentity: includeDeviceIdentity
        )
    }
}

private extension String {
    nonisolated var trimmedNonEmpty: String? {
        let trimmed = self.trimmingCharacters(in: .whitespacesAndNewlines)
        return trimmed.isEmpty ? nil : trimmed
    }
}
