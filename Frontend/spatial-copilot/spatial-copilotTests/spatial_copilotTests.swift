import Foundation
import Testing
import OpenClawProtocol
import RosClawBridge
@testable import spatial_copilot

struct spatial_copilotTests {

    @MainActor
    @Test func applySnapshotMapsFollowerStateIntoRobotState() async throws {
        let coordinator = AppCoordinator()
        let state = GatewayFollowerState(
            jointOrder: ["shoulder_pan", "shoulder_lift"],
            jointPositions: [
                "shoulder_pan": 0.25,
                "shoulder_lift": -0.40,
            ],
            gripperPosition: 0.018,
            gripperOpenFraction: 1.0,
            toolPose: RobotToolPose(
                frameID: "follower/base_link",
                position: RobotVector3(x: 0.1, y: 0.2, z: 0.3),
                orientation: RobotQuaternion(x: 0.0, y: 0.707107, z: 0.0, w: 0.707107)
            ),
            workspaceBounds: WorkspaceBounds(
                xMin: -0.2, xMax: 0.4, yMin: -0.3, yMax: 0.3, zMin: 0.05, zMax: 0.45
            ),
            namedPoseNames: ["home", "setup"],
            currentNamedPose: "home",
            lastResolvedCartesianGoal: GatewayResolvedCartesianGoal(
                accepted: true,
                reason: "resolved to the nearest safe reachable pose",
                requestedPose: RobotToolPose(
                    frameID: "follower/base_link",
                    position: RobotVector3(x: 0.3, y: 0.1, z: 0.2),
                    orientation: RobotQuaternion(x: 0, y: 0.707107, z: 0, w: 0.707107)
                ),
                resolvedPose: RobotToolPose(
                    frameID: "follower/base_link",
                    position: RobotVector3(x: 0.28, y: 0.1, z: 0.21),
                    orientation: RobotQuaternion(x: 0, y: 0.707107, z: 0, w: 0.707107)
                ),
                jointPositions: [0.1, -0.2, 0.3],
                positionErrorM: 0.022,
                orientationErrorRad: 0.05,
                executed: true
            )
        )
        let snapshot = GatewayRobotSnapshot(
            executorAgentID: "robot-executor",
            executorSessionKey: "visionos-robot",
            nodeID: "visionos-node",
            capabilities: .followerDefault,
            state: state,
            stateSummary: nil,
            estopLatched: false
        )

        coordinator.connection.applySnapshot(snapshot)

        #expect(coordinator.connection.remoteAgentID == "robot-executor")
        #expect(coordinator.connection.remoteSessionKey == "visionos-robot")
        #expect(coordinator.connection.remoteNodeID == "visionos-node")
        #expect(coordinator.state.robotState.connectionState.rawValue == ConnectionState.connected.rawValue)
        #expect(coordinator.state.robotState.enabled == true)
        #expect(coordinator.state.robotState.jointOrder == ["shoulder_pan", "shoulder_lift"])
        #expect(coordinator.state.robotState.jointPositions["shoulder_pan"] == 0.25)
        #expect(coordinator.state.robotState.gripperOpenFraction == 1.0)
        #expect(coordinator.state.robotState.gripperState.rawValue == GripperState.open.rawValue)
        #expect(coordinator.connection.workspaceBounds == WorkspaceBounds(xMin: -0.2, xMax: 0.4, yMin: -0.3, yMax: 0.3, zMin: 0.05, zMax: 0.45))
        #expect(coordinator.connection.supportedPresetSummary == ["home", "setup"])
        #expect(coordinator.state.currentNamedPose == "home")
        #expect(coordinator.state.lastResolvedCartesianGoal?.accepted == true)
    }

    @MainActor
    @Test func applySnapshotReflectsLatchedEstop() async throws {
        let coordinator = AppCoordinator()
        let snapshot = GatewayRobotSnapshot(
            executorAgentID: "robot-executor",
            executorSessionKey: "visionos-robot",
            nodeID: "visionos-node",
            capabilities: .followerDefault,
            state: nil,
            stateSummary: nil,
            estopLatched: true
        )

        coordinator.connection.applySnapshot(snapshot)

        #expect(coordinator.state.robotState.estopLatched == true)
        #expect(coordinator.state.robotState.enabled == false)
        #expect(coordinator.state.robotState.mode.rawValue == RobotMode.faulted.rawValue)
        #expect(coordinator.state.robotState.fault == "Emergency stop latched")
    }

    @MainActor
    @Test func applySnapshotKeepsGatewayConnectedWhenRobotStateIsUnavailable() async throws {
        let coordinator = AppCoordinator()
        let snapshot = GatewayRobotSnapshot(
            executorAgentID: "robot-executor",
            executorSessionKey: "visionos-robot",
            nodeID: "device-fingerprint",
            capabilities: .followerDefault,
            state: nil,
            stateSummary: "Robot state unavailable.",
            estopLatched: false
        )

        coordinator.connection.applySnapshot(snapshot)

        #expect(coordinator.state.robotState.connectionState.rawValue == ConnectionState.connected.rawValue)
        #expect(coordinator.state.robotState.enabled == false)
        #expect(coordinator.state.robotState.mode.rawValue == RobotMode.unknown.rawValue)
        #expect(coordinator.state.robotState.fault == "Robot state unavailable.")
        #expect(coordinator.connection.remoteStateSummary == "Robot state unavailable.")
    }

    @MainActor
    @Test func applySnapshotDerivesLocalToolPoseAndGripperFractionWhenBackendOmitsThem() async throws {
        let coordinator = AppCoordinator()
        let state = GatewayFollowerState(
            jointOrder: ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
            jointPositions: [
                "shoulder_pan": -0.5577381829450011,
                "shoulder_lift": -1.763741516630753,
                "elbow_flex": 1.5473974071527747,
                "wrist_flex": 0.9911935795941422,
                "wrist_roll": -1.6594053503576858,
                "gripper": 0.9885706612647418,
            ],
            gripperPosition: 0.9885706612647418,
            gripperOpenFraction: nil,
            toolPose: nil,
            toolPoseStatus: GatewayToolPoseStatus(
                stage: "tool_pose_unavailable",
                summary: "Joint states and kinematics are ready, but forward kinematics returned no tool pose.",
                jointStateReceived: true,
                jointCount: 6,
                robotDescriptionReceived: true,
                kinematicsReady: true,
                missingJointNames: [],
                baseFrame: "follower/base_link",
                toolFrame: "follower/gripper_frame_link"
            )
        )
        let snapshot = GatewayRobotSnapshot(
            executorAgentID: "robot-executor",
            executorSessionKey: "visionos-robot",
            nodeID: "visionos-node",
            capabilities: .followerDefault,
            state: state,
            stateSummary: nil,
            estopLatched: false
        )

        coordinator.connection.applySnapshot(snapshot)

        #expect(coordinator.state.robotState.gripperPosition == 0.9885706612647418)
        #expect(coordinator.state.robotState.gripperOpenFraction == 0.9885706612647418)
        #expect(coordinator.state.robotState.toolPose?.position != nil)
        #expect(coordinator.state.toolPoseStatus?.stage == "local_fk_fallback")
        #expect(coordinator.connection.remotePoseFlowSummary?.contains("local SO-101 forward kinematics") == true)
    }

    @MainActor
    @Test func applyCommandResultLatchesEstopAndDisablesExecution() async throws {
        let coordinator = AppCoordinator()
        let result = GatewayRobotCommandResult(
            intentID: "intent-1",
            ok: true,
            summary: "Emergency stop latched",
            state: nil,
            errorCode: nil,
            estopLatched: true
        )

        coordinator.connection.applyCommandResult(result)

        #expect(coordinator.state.robotState.estopLatched == true)
        #expect(coordinator.state.robotState.enabled == false)
        #expect(coordinator.state.robotState.actionStatus.rawValue == ActionStatus.cancelled.rawValue)
        #expect(coordinator.state.robotState.mode.rawValue == RobotMode.faulted.rawValue)
        #expect(coordinator.state.robotState.fault == "Emergency stop latched")
    }

    @MainActor
    @Test func enteringImmersiveModeClearsPendingPanelPreviewState() async throws {
        let coordinator = AppCoordinator()
        coordinator.commandFlow.pendingCommand = .closeGripper
        coordinator.commandFlow.pendingPreview = CommandPreview(
            parsedCommandName: "Close Gripper",
            targets: [],
            validationResult: .allowed,
            subsystemsTouched: ["gripper"],
            requiresConfirmation: true,
            canExecute: true
        )
        coordinator.commandFlow.pendingClarification = PendingCartesianClarification(
            question: "How far?",
            draft: PendingCartesianClarificationDraft(
                x: 1,
                y: 2,
                z: 3,
                referenceFrame: .baseAbsolute,
                source: .voice
            )
        )
        coordinator.commandFlow.appPhase = .awaitingConfirmation

        coordinator.commandFlow.enterImmersiveMode()

        #expect(coordinator.commandFlow.pendingCommand == nil)
        #expect(coordinator.commandFlow.pendingPreview == nil)
        #expect(coordinator.commandFlow.pendingClarification == nil)
        #expect(coordinator.commandFlow.appPhase == .idle)
    }
    @Test func gatewayEndpointPrefersExplicitGatewayURL() throws {
        let configuration = try GatewayRobotEndpointConfiguration.from(environment: [
            "OPENCLAW_GATEWAY_URL": "gateway.example.com",
            "OPENCLAW_GATEWAY_TOKEN": "token-123",
            "OPENCLAW_OPERATOR_CLIENT_ID": "OPENCLAW-IOS",
            "OPENCLAW_NODE_CLIENT_ID": "MOLTBOT-IOS",
        ])
        #expect(configuration.gatewayURL.absoluteString == "wss://gateway.example.com")
        #expect(configuration.authToken == "token-123")
        #expect(configuration.operatorClientID == "openclaw-ios")
        #expect(configuration.nodeClientID == "moltbot-ios")
    }

    @Test func gatewayEndpointIgnoresInvalidClientIDOverrides() throws {
        let configuration = try GatewayRobotEndpointConfiguration.from(environment: [
            "OPENCLAW_GATEWAY_URL": "gateway.example.com",
            "OPENCLAW_OPERATOR_CLIENT_ID": "visionos-operator",
            "OPENCLAW_NODE_ID": "visionos-node",
        ])
        #expect(configuration.operatorClientID == "openclaw-ios")
        #expect(configuration.nodeClientID == "openclaw-ios")
    }

    @Test func operatorScopesMatchOracleDefaults() {
        #expect(GatewayOperatorConnectScopes.oracle == ["operator.admin", "operator.read", "operator.write", "operator.approvals", "operator.pairing"])
    }

    @Test func bootstrapPayloadNormalizesOracleCommands() throws {
        let json = """
        {"executorAgentId":"robot-executor","executorSessionKey":"robot-main","supportedCommands":["open_gripper","move_to_cartesian","move_to_preset","home_robot"],"presets":["home","rest"],"workspaceBounds":{"xMin":-0.2,"xMax":0.4,"yMin":-0.3,"yMax":0.3,"zMin":0.0,"zMax":0.5},"state":{"joint_order":["shoulder_pan"],"joint_positions":{"shoulder_pan":0.1},"gripper_position":0.02,"gripper_open_fraction":0.8,"tool_pose":null,"estop_latched":true}}
        """
        let payload = try JSONDecoder().decode(GatewayRosclawBootstrapPayload.self, from: Data(json.utf8))
        #expect(payload.executorAgentID == "robot-executor")
        #expect(payload.capabilities.supportedCommands.contains("openGripper"))
        #expect(payload.capabilities.supportedCommands.contains("moveToCartesian"))
        #expect(payload.capabilities.supportedCommands.contains("moveToPreset"))
        #expect(payload.capabilities.supportedCommands.contains("homeRobot"))
        #expect(payload.capabilities.supportedCommands.contains("emergencyStop"))
        #expect(payload.capabilities.supportedPresets == ["home", "rest"])
        #expect(payload.estopLatched == true)
    }

    @Test func bootstrapPayloadFallsBackToStateNamedPosesAndWorkspaceBounds() throws {
        let json = """
        {"executorAgentId":"robot-executor","state":{"joint_order":["shoulder_pan"],"joint_positions":{"shoulder_pan":0.1},"workspace_bounds":{"xMin":-0.25,"xMax":0.45,"yMin":-0.35,"yMax":0.35,"zMin":0.02,"zMax":0.42},"named_pose_names":["home","zero"]}}
        """
        let payload = try JSONDecoder().decode(GatewayRosclawBootstrapPayload.self, from: Data(json.utf8))
        #expect(payload.capabilities.supportedPresets == ["home", "zero"])
        #expect(payload.capabilities.supportedCommands.contains("moveToPreset"))
        #expect(payload.capabilities.supportedCommands.contains("homeRobot"))
        #expect(payload.capabilities.workspaceBounds == WorkspaceBounds(xMin: -0.25, xMax: 0.45, yMin: -0.35, yMax: 0.35, zMin: 0.02, zMax: 0.42))
    }

    @Test func followerStateDecodesToolPoseStatusPipelineMetadata() throws {
        let json = """
        {
          "joint_order": ["shoulder_pan"],
          "joint_positions": {"shoulder_pan": 0.1},
          "tool_pose": null,
          "tool_pose_status": {
            "stage": "waiting_for_robot_description",
            "summary": "Waiting for robot_description on /follower/robot_description before forward kinematics can start.",
            "joint_state_received": true,
            "joint_count": 1,
            "robot_description_received": false,
            "kinematics_ready": false,
            "missing_joint_names": [],
            "base_frame": "follower/base_link",
            "tool_frame": "follower/gripper_frame_link"
          }
        }
        """
        let state = try JSONDecoder().decode(GatewayFollowerState.self, from: Data(json.utf8))
        #expect(state.toolPoseStatus?.stage == "waiting_for_robot_description")
        #expect(state.toolPoseStatus?.jointStateReceived == true)
        #expect(state.toolPoseStatus?.jointCount == 1)
        #expect(state.toolPoseStatus?.robotDescriptionReceived == false)
        #expect(state.toolPoseStatus?.toolFrame == "follower/gripper_frame_link")
    }

    @Test func deviceAuthPayloadMatchesOracleVector() {
        let payload = GatewayDeviceAuthPayload.buildV3(
            deviceId: "dev-1", clientId: "openclaw-ios", clientMode: "ui", role: "operator",
            scopes: ["operator.admin", "operator.read"], signedAtMs: 1_700_000_000_000,
            token: "tok-123", nonce: "nonce-abc", platform: "  visionOS  ", deviceFamily: "  Vision  "
        )
        #expect(payload == "v3|dev-1|openclaw-ios|ui|operator|operator.admin,operator.read|1700000000000|tok-123|nonce-abc|visionos|vision")
    }

    @Test func legacyClientIDFallbackMatchesOracleBehavior() {
        struct StubError: LocalizedError { let message: String; var errorDescription: String? { message } }
        let fallback = GatewayVisionOSClientID.legacyFallback(
            currentClientID: "openclaw-ios",
            error: StubError(message: "invalid connect params: at /client/id: must be equal to constant")
        )
        #expect(fallback == "moltbot-ios")
        #expect(GatewayVisionOSClientID.legacyFallback(currentClientID: "moltbot-ios", error: StubError(message: "invalid connect params: at /client/id: must be equal to constant")) == nil)
    }

    @Test func typedGatewayMethodsDisableOnAdminScopeErrors() {
        struct StubError: LocalizedError { let message: String; var errorDescription: String? { message } }
        #expect(GatewayRobotExecutor.shouldDisableTypedGatewayMethods(for: StubError(message: "missing scope: operator.admin")))
        #expect(GatewayRobotExecutor.shouldDisableTypedGatewayMethods(for: StubError(message: "method not found: rosclaw.bootstrap")))
        #expect(GatewayRobotExecutor.shouldDisableTypedGatewayMethods(for: StubError(message: "gateway closed")) == false)
    }

    @Test func selectPendingOwnNodePairingOnlyApprovesPureNodeRequestsForCurrentDevice() {
        let requests = [
            try! JSONDecoder().decode(GatewayPendingDevicePairingRequest.self, from: Data("""
            {"requestId":"req-operator","deviceId":"device-1","displayName":"VisionOS Copilot","role":"operator","roles":["operator"],"scopes":["operator.admin"],"clientMode":"ui","ts":20}
            """.utf8)),
            try! JSONDecoder().decode(GatewayPendingDevicePairingRequest.self, from: Data("""
            {"requestId":"req-node","deviceId":"device-1","displayName":"VisionOS Copilot","role":"node","roles":["node"],"scopes":[],"clientMode":"node","ts":30}
            """.utf8)),
            try! JSONDecoder().decode(GatewayPendingDevicePairingRequest.self, from: Data("""
            {"requestId":"req-other","deviceId":"device-2","displayName":"VisionOS Copilot","role":"node","roles":["node"],"scopes":[],"clientMode":"node","ts":40}
            """.utf8)),
        ]
        let selected = GatewayRobotExecutor.selectPendingOwnNodePairing(from: requests, deviceID: "device-1", displayName: "VisionOS Copilot")
        #expect(selected?.requestId == "req-node")
        #expect(GatewayRobotExecutor.isPureNodePairingRequest(requests[0]) == false)
        #expect(GatewayRobotExecutor.isPureNodePairingRequest(requests[1]) == true)
    }

    @Test func runtimeNodeIdentityUsesDeviceFingerprintWhenEnabled() {
        let identity = GatewayDeviceIdentity(deviceId: "device-fingerprint", publicKey: "pub", privateKey: "priv", createdAtMs: 0)
        #expect(GatewayRuntimeNodeIdentity.resolve(includeDeviceIdentity: true, clientID: "openclaw-ios", deviceID: identity.deviceId) == "device-fingerprint")
        #expect(GatewayRuntimeNodeIdentity.resolve(includeDeviceIdentity: false, clientID: "openclaw-ios", deviceID: identity.deviceId) == "openclaw-ios")
    }

    @Test func connectFrameEncodeCapsAndScopesAsArrays() throws {
        let data = try GatewayConnectFrameEncoder.encode(
            requestID: "req-1", nonce: "nonce-1",
            options: GatewayConnectOptions(role: "operator", scopes: ["operator.read", "operator.write"], caps: ["tool-events"], commands: [], permissions: [:], clientId: "openclaw-ios", clientMode: "ui", clientDisplayName: "VisionOS Copilot", includeDeviceIdentity: false),
            appVersion: "dev",
            identity: GatewayConnectIdentitySnapshot(platform: "visionOS 2.0.0", displayName: "VisionOS Copilot", instanceId: "instance-1", deviceFamily: "Vision", modelIdentifier: nil, locale: "en-US", userAgent: "visionOS test"),
            deviceIdentity: nil, authToken: "token-123", authBootstrapToken: nil, authPassword: nil, signatureToken: "token-123"
        )
        let json = try JSONSerialization.jsonObject(with: data) as? [String: Any]
        let params = json?["params"] as? [String: Any]
        #expect(params?["caps"] as? [String] == ["tool-events"])
        #expect(params?["scopes"] as? [String] == ["operator.read", "operator.write"])
        #expect(params?["locale"] as? String == "en-US")
        #expect(params?["userAgent"] as? String == "visionOS test")
    }
    @Test func robotCartesianTargetNormalizesMetricUnitsIntoCentimetersAndMeters() {
        let target = RobotCartesianTarget(x: 150.0, y: 25.0, z: 500.0, unit: .millimeters, referenceFrame: .toolRelative, source: .voice)
        #expect(target.requestedCentimeters == RobotVector3(x: 15.0, y: 2.5, z: 50.0))
        #expect(target.requestedMeters == RobotVector3(x: 0.15, y: 0.025, z: 0.5))
        #expect(target.centimeterSummary == "(15.000 cm, 2.500 cm, 50.000 cm)")
    }

    @Test func toolRelativeSafeRangeUsesCurrentToolPoseFromRobotOdometry() {
        let gate = SafetyGate(workspaceBounds: WorkspaceBounds(xMin: -0.2, xMax: 0.4, yMin: -0.3, yMax: 0.3, zMin: 0.05, zMax: 0.45))
        var state = RobotState()
        state.toolPose = RobotToolPose(frameID: "follower/base_link", position: RobotVector3(x: 0.1, y: -0.05, z: 0.2), orientation: RobotQuaternion(x: 0.0, y: 0.707107, z: 0.0, w: 0.707107))
        let range = gate.safeRange(for: .toolRelative, robotState: state)
        let epsilon = 0.0001
        #expect(abs((range?.x.minCm ?? .greatestFiniteMagnitude) - (-30.0)) < epsilon)
        #expect(abs((range?.x.maxCm ?? .greatestFiniteMagnitude) - 30.0) < epsilon)
        #expect(abs((range?.y.minCm ?? .greatestFiniteMagnitude) - (-25.0)) < epsilon)
        #expect(abs((range?.y.maxCm ?? .greatestFiniteMagnitude) - 35.0) < epsilon)
        #expect(abs((range?.z.minCm ?? .greatestFiniteMagnitude) - (-15.0)) < epsilon)
        #expect(abs((range?.z.maxCm ?? .greatestFiniteMagnitude) - 25.0) < epsilon)
    }

    @Test func toolRelativeCartesianMoveRequiresLiveToolPose() {
        let gate = SafetyGate()
        var state = RobotState()
        state.connectionState = .connected; state.enabled = true; state.mode = .idle
        let command = RobotCommand.moveToCartesian(target: RobotCartesianTarget(x: 5.0, y: 0.0, z: 0.0, unit: .centimeters, referenceFrame: .toolRelative, source: .voice))
        let result = gate.validate(command, robotState: state)
        #expect(result.isAllowed == false)
        #expect(result.blockReason?.contains("Live gripper pose is unavailable") == true)
    }

    @Test func cartesianClarificationDraftFinalizesWhenUserSuppliesUnits() {
        let draft = PendingCartesianClarificationDraft(x: 12.0, y: 0.0, z: -3.0, referenceFrame: .toolRelative, source: .voice)
        let centimeters = CartesianVoiceCommandSupport.finalizeDraft(draft, with: "centimeters")
        let meters = CartesianVoiceCommandSupport.finalizeDraft(draft, with: "use meters please")
        let unresolved = CartesianVoiceCommandSupport.finalizeDraft(draft, with: "yes")
        #expect(centimeters == RobotCartesianTarget(x: 12.0, y: 0.0, z: -3.0, unit: .centimeters, referenceFrame: .toolRelative, source: .voice))
        #expect(meters?.unit == .meters)
        #expect(unresolved == nil)
    }

    @MainActor
    @Test func parserTurnsMissingUnitCartesianPayloadIntoClarification() throws {
        let coordinator = AppCoordinator()
        let result = try coordinator.commandFlow.testingDecodeParsedInput("""
        {"kind":"command","command":"moveToCartesian","target":{"x":12.0,"y":0.0,"z":-3.0,"referenceFrame":"toolRelative","source":"voice"}}
        """)
        #expect(result == .clarification(PendingCartesianClarification(
            question: "Do you mean millimeters, centimeters, or meters?",
            draft: PendingCartesianClarificationDraft(x: 12.0, y: 0.0, z: -3.0, referenceFrame: .toolRelative, source: .voice))))
    }

    @MainActor
    @Test func pendingClarificationResolvesLocallyBeforeGatewayReadyCheck() async {
        let coordinator = AppCoordinator()
        coordinator.connection.remoteBootState = .faulted
        coordinator.state.robotState.connectionState = .connected
        coordinator.state.robotState.enabled = true
        coordinator.state.robotState.mode = .idle
        coordinator.commandFlow.pendingClarification = PendingCartesianClarification(
            question: "Do you mean millimeters, centimeters, or meters?",
            draft: PendingCartesianClarificationDraft(x: 12.0, y: 0.0, z: -3.0, referenceFrame: .toolRelative, source: .voice))
        await coordinator.commandFlow.handleInput("centimeters")
        #expect(coordinator.commandFlow.pendingClarification == nil)
        #expect(coordinator.commandFlow.pendingCommand == .moveToCartesian(target: RobotCartesianTarget(x: 12.0, y: 0.0, z: -3.0, unit: .centimeters, referenceFrame: .toolRelative, source: .voice)))
        #expect(coordinator.commandFlow.appPhase == .awaitingConfirmation)
    }

    @MainActor
    @Test func clarificationFlowStopsAfterRepeatedInvalidUnitAnswers() async {
        let coordinator = AppCoordinator()
        let clarification = PendingCartesianClarification(
            question: "Do you mean millimeters, centimeters, or meters?",
            draft: PendingCartesianClarificationDraft(x: 12.0, y: 0.0, z: -3.0, referenceFrame: .toolRelative, source: .voice))
        coordinator.commandFlow.pendingClarification = clarification
        await coordinator.commandFlow.testingResolvePendingClarification(with: "yes", clarification: clarification)
        #expect(coordinator.commandFlow.pendingClarification?.remainingAttempts == 1)
        #expect(coordinator.commandFlow.appPhase == .awaitingClarification)
        guard let retry = coordinator.commandFlow.pendingClarification else { #expect(Bool(false)); return }
        await coordinator.commandFlow.testingResolvePendingClarification(with: "that sounds good", clarification: retry)
        #expect(coordinator.commandFlow.pendingClarification == nil)
        #expect(coordinator.commandFlow.pendingCommand == nil)
        #expect(coordinator.commandFlow.appPhase == .idle)
        #expect(coordinator.commandFlow.transcript.last?.text == "I still need the Cartesian unit. Please restate the move with millimeters, centimeters, or meters.")
    }

    @Test func rosclawPromptOracleUsesManifestFirstStateResolution() {
        #expect(GatewayRosclawPromptOracle.manifestResolutionGuidance.contains("ros2_list_topics"))
        #expect(GatewayRosclawPromptOracle.stateResolutionGuidance.contains("`agent_state`"))
        #expect(GatewayRosclawPromptOracle.stateResolutionGuidance.contains("`joint_states`"))
        #expect(GatewayRosclawPromptOracle.stateResolutionGuidance.contains("joint_positions"))
    }

    @Test func rosclawPromptOracleAllowsFallbackControlInterfaces() {
        let openGuidance = GatewayRosclawPromptOracle.gripperCommandGuidance(open: true)
        let closeGuidance = GatewayRosclawPromptOracle.gripperCommandGuidance(open: false)
        #expect(openGuidance.contains("/follower/open_gripper"))
        #expect(openGuidance.contains("gripper_command"))
        #expect(openGuidance.contains("state:null"))
        #expect(closeGuidance.contains("/follower/close_gripper"))
        #expect(GatewayRosclawPromptOracle.namedPoseDiscoveryGuidance.contains("list_named_poses"))
        #expect(GatewayRosclawPromptOracle.namedPoseCommandGuidance(name: "home").contains("move_to_named_pose"))
        #expect(GatewayRosclawPromptOracle.cartesianCommandGuidance.contains("convert_cartesian_coordinates"))
        #expect(GatewayRosclawPromptOracle.cartesianCommandGuidance.contains("move_to_cartesian_goal"))
        #expect(GatewayRosclawPromptOracle.estopCommandGuidance.contains("/rosclaw/estop"))
    }

    @Test func voiceCartesianPromptUsesCentimeterConversionAndToolFrame() async throws {
        let executor = GatewayRobotExecutor()
        let prompt = try await executor.testingPrompt(for: .moveToCartesian(target: RobotCartesianTarget(x: 25.0, y: 0.0, z: -5.0, unit: .centimeters, referenceFrame: .toolRelative, source: .voice)))
        #expect(prompt.contains("requestedPositionUser"))
        #expect(prompt.contains("requestedPositionCentimeters"))
        #expect(prompt.contains("Because this command came from a spoken metric request"))
        #expect(prompt.contains(FollowerTopics.toolFrame))
        #expect(prompt.contains("requested (25.000 cm, 0.000 cm, -5.000 cm) [Tool-relative]"))
    }

    @Test func spatialCartesianPromptUsesMetersAndBaseFrame() async throws {
        let executor = GatewayRobotExecutor()
        let prompt = try await executor.testingPrompt(for: .moveToCartesian(target: RobotCartesianTarget(x: 0.2, y: -0.1, z: 0.3, unit: .meters, referenceFrame: .baseAbsolute, source: .spatial)))
        #expect(prompt.contains("requestedPositionMeters"))
        #expect(prompt.contains("This request is already expressed in meters"))
        #expect(prompt.contains(FollowerTopics.baseFrame))
        #expect(prompt.contains("requested (0.200 m, -0.100 m, 0.300 m) [Base absolute]"))
    }

    @Test func spatialTargetingMathMapsWorldCoordinatesIntoRobotFrame() throws {
        let basePosition = SIMD3<Float>(0.3, 0.1, 0.4)
        let worldPosition = SIMD3<Float>(0.8, -0.2, 1.2)
        let expected = worldPosition - basePosition
        let calibration = try #require(
            SpatialTargetingMath.solveCalibration(
                baseWorldPoint: basePosition,
                gripperWorldPoint: worldPosition,
                toolPoseInRobotFrame: expected,
                surfaceNormal: SIMD3<Float>(0, 0, 1)
            )
        )
        let robotPosition = SpatialTargetingMath.robotPosition(
            fromWorldPosition: worldPosition,
            using: calibration.worldToRobotTransform
        )
        let epsilon: Float = 0.0001
        #expect(abs(robotPosition.x - expected.x) < epsilon)
        #expect(abs(robotPosition.y - expected.y) < epsilon)
        #expect(abs(robotPosition.z - expected.z) < epsilon)
    }
}
