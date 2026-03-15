//
//  robot_copilot_demoTests.swift
//  robot-copilot-demoTests
//
//  Created by Marcus Arnett on 3/12/26.
//

import Foundation
import Testing
import OpenClawProtocol
import RosClawBridge
@testable import robot_copilot_demo

struct robot_copilot_demoTests {

    @MainActor
    @Test func applySnapshotMapsFollowerStateIntoRobotState() async throws {
        let model = AppModel()
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
                xMin: -0.2,
                xMax: 0.4,
                yMin: -0.3,
                yMax: 0.3,
                zMin: 0.05,
                zMax: 0.45
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

        model.applySnapshot(snapshot)

        #expect(model.remoteAgentID == "robot-executor")
        #expect(model.remoteSessionKey == "visionos-robot")
        #expect(model.remoteNodeID == "visionos-node")
        #expect(model.robotState.connectionState.rawValue == ConnectionState.connected.rawValue)
        #expect(model.robotState.enabled == true)
        #expect(model.robotState.jointOrder == ["shoulder_pan", "shoulder_lift"])
        #expect(model.robotState.jointPositions["shoulder_pan"] == 0.25)
        #expect(model.robotState.gripperOpenFraction == 1.0)
        #expect(model.robotState.gripperState.rawValue == GripperState.open.rawValue)
        #expect(model.workspaceBounds == WorkspaceBounds(xMin: -0.2, xMax: 0.4, yMin: -0.3, yMax: 0.3, zMin: 0.05, zMax: 0.45))
        #expect(model.supportedPresetSummary == ["home", "setup"])
        #expect(model.currentNamedPose == "home")
        #expect(model.lastResolvedCartesianGoal?.accepted == true)
    }

    @MainActor
    @Test func applySnapshotReflectsLatchedEstop() async throws {
        let model = AppModel()
        let snapshot = GatewayRobotSnapshot(
            executorAgentID: "robot-executor",
            executorSessionKey: "visionos-robot",
            nodeID: "visionos-node",
            capabilities: .followerDefault,
            state: nil,
            stateSummary: nil,
            estopLatched: true
        )

        model.applySnapshot(snapshot)

        #expect(model.robotState.estopLatched == true)
        #expect(model.robotState.enabled == false)
        #expect(model.robotState.mode.rawValue == RobotMode.faulted.rawValue)
        #expect(model.robotState.fault == "Emergency stop latched")
    }

    @MainActor
    @Test func applySnapshotKeepsGatewayConnectedWhenRobotStateIsUnavailable() async throws {
        let model = AppModel()
        let snapshot = GatewayRobotSnapshot(
            executorAgentID: "robot-executor",
            executorSessionKey: "visionos-robot",
            nodeID: "device-fingerprint",
            capabilities: .followerDefault,
            state: nil,
            stateSummary: "Robot state unavailable.",
            estopLatched: false
        )

        model.applySnapshot(snapshot)

        #expect(model.robotState.connectionState.rawValue == ConnectionState.connected.rawValue)
        #expect(model.robotState.enabled == false)
        #expect(model.robotState.mode.rawValue == RobotMode.unknown.rawValue)
        #expect(model.robotState.fault == "Robot state unavailable.")
        #expect(model.remoteStateSummary == "Robot state unavailable.")
    }

    @MainActor
    @Test func applyCommandResultLatchesEstopAndDisablesExecution() async throws {
        let model = AppModel()
        let result = GatewayRobotCommandResult(
            intentID: "intent-1",
            ok: true,
            summary: "Emergency stop latched",
            state: nil,
            errorCode: nil,
            estopLatched: true
        )

        model.applyCommandResult(result)

        #expect(model.robotState.estopLatched == true)
        #expect(model.robotState.enabled == false)
        #expect(model.robotState.actionStatus.rawValue == ActionStatus.cancelled.rawValue)
        #expect(model.robotState.mode.rawValue == RobotMode.faulted.rawValue)
        #expect(model.robotState.fault == "Emergency stop latched")
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
        #expect(
            GatewayOperatorConnectScopes.oracle ==
                ["operator.admin", "operator.read", "operator.write", "operator.approvals", "operator.pairing"]
        )
    }

    @Test func bootstrapPayloadNormalizesOracleCommands() throws {
        let json = """
        {
          "executorAgentId": "robot-executor",
          "executorSessionKey": "robot-main",
          "supportedCommands": ["open_gripper", "move_to_cartesian", "move_to_preset", "home_robot"],
          "presets": ["home", "rest"],
          "workspaceBounds": {
            "xMin": -0.2,
            "xMax": 0.4,
            "yMin": -0.3,
            "yMax": 0.3,
            "zMin": 0.0,
            "zMax": 0.5
          },
          "state": {
            "joint_order": ["shoulder_pan"],
            "joint_positions": {"shoulder_pan": 0.1},
            "gripper_position": 0.02,
            "gripper_open_fraction": 0.8,
            "tool_pose": null,
            "estop_latched": true
          }
        }
        """

        let payload = try JSONDecoder().decode(
            GatewayRosclawBootstrapPayload.self,
            from: Data(json.utf8)
        )

        #expect(payload.executorAgentID == "robot-executor")
        #expect(payload.executorSessionKey == "robot-main")
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
        {
          "executorAgentId": "robot-executor",
          "state": {
            "joint_order": ["shoulder_pan"],
            "joint_positions": {"shoulder_pan": 0.1},
            "workspace_bounds": {
              "xMin": -0.25,
              "xMax": 0.45,
              "yMin": -0.35,
              "yMax": 0.35,
              "zMin": 0.02,
              "zMax": 0.42
            },
            "named_pose_names": ["home", "zero"]
          }
        }
        """

        let payload = try JSONDecoder().decode(
            GatewayRosclawBootstrapPayload.self,
            from: Data(json.utf8)
        )

        #expect(payload.capabilities.supportedPresets == ["home", "zero"])
        #expect(payload.capabilities.supportedCommands.contains("moveToPreset"))
        #expect(payload.capabilities.supportedCommands.contains("homeRobot"))
        #expect(payload.capabilities.workspaceBounds == WorkspaceBounds(xMin: -0.25, xMax: 0.45, yMin: -0.35, yMax: 0.35, zMin: 0.02, zMax: 0.42))
    }

    @Test func deviceAuthPayloadMatchesOracleVector() {
        let payload = GatewayDeviceAuthPayload.buildV3(
            deviceId: "dev-1",
            clientId: "openclaw-ios",
            clientMode: "ui",
            role: "operator",
            scopes: ["operator.admin", "operator.read"],
            signedAtMs: 1_700_000_000_000,
            token: "tok-123",
            nonce: "nonce-abc",
            platform: "  visionOS  ",
            deviceFamily: "  Vision  "
        )

        #expect(
            payload ==
                "v3|dev-1|openclaw-ios|ui|operator|operator.admin,operator.read|1700000000000|tok-123|nonce-abc|visionos|vision"
        )
    }

    @Test func legacyClientIDFallbackMatchesOracleBehavior() {
        struct StubError: LocalizedError {
            let message: String
            var errorDescription: String? { message }
        }

        let fallback = GatewayVisionOSClientID.legacyFallback(
            currentClientID: "openclaw-ios",
            error: StubError(message: "invalid connect params: at /client/id: must be equal to constant")
        )

        #expect(fallback == "moltbot-ios")
        #expect(
            GatewayVisionOSClientID.legacyFallback(
                currentClientID: "moltbot-ios",
                error: StubError(message: "invalid connect params: at /client/id: must be equal to constant")
            ) == nil
        )
    }

    @Test func typedGatewayMethodsDisableOnAdminScopeErrors() {
        struct StubError: LocalizedError {
            let message: String
            var errorDescription: String? { message }
        }

        #expect(
            GatewayRobotExecutor.shouldDisableTypedGatewayMethods(
                for: StubError(message: "missing scope: operator.admin")
            )
        )
        #expect(
            GatewayRobotExecutor.shouldDisableTypedGatewayMethods(
                for: StubError(message: "method not found: rosclaw.bootstrap")
            )
        )
        #expect(
            GatewayRobotExecutor.shouldDisableTypedGatewayMethods(
                for: StubError(message: "gateway closed")
            ) == false
        )
    }

    @Test func selectPendingOwnNodePairingOnlyApprovesPureNodeRequestsForCurrentDevice() {
        let requests = [
            try! JSONDecoder().decode(
                GatewayPendingDevicePairingRequest.self,
                from: Data(
                    """
                    {"requestId":"req-operator","deviceId":"device-1","displayName":"VisionOS Copilot","role":"operator","roles":["operator"],"scopes":["operator.admin"],"clientMode":"ui","ts":20}
                    """.utf8
                )
            ),
            try! JSONDecoder().decode(
                GatewayPendingDevicePairingRequest.self,
                from: Data(
                    """
                    {"requestId":"req-node","deviceId":"device-1","displayName":"VisionOS Copilot","role":"node","roles":["node"],"scopes":[],"clientMode":"node","ts":30}
                    """.utf8
                )
            ),
            try! JSONDecoder().decode(
                GatewayPendingDevicePairingRequest.self,
                from: Data(
                    """
                    {"requestId":"req-other","deviceId":"device-2","displayName":"VisionOS Copilot","role":"node","roles":["node"],"scopes":[],"clientMode":"node","ts":40}
                    """.utf8
                )
            ),
        ]

        let selected = GatewayRobotExecutor.selectPendingOwnNodePairing(
            from: requests,
            deviceID: "device-1",
            displayName: "VisionOS Copilot"
        )

        #expect(selected?.requestId == "req-node")
        #expect(GatewayRobotExecutor.isPureNodePairingRequest(requests[0]) == false)
        #expect(GatewayRobotExecutor.isPureNodePairingRequest(requests[1]) == true)
    }

    @Test func runtimeNodeIdentityUsesDeviceFingerprintWhenEnabled() {
        let identity = GatewayDeviceIdentity(
            deviceId: "device-fingerprint",
            publicKey: "pub",
            privateKey: "priv",
            createdAtMs: 0
        )

        #expect(
            GatewayRuntimeNodeIdentity.resolve(
                includeDeviceIdentity: true,
                clientID: "openclaw-ios",
                deviceID: identity.deviceId
            ) == "device-fingerprint"
        )
        #expect(
            GatewayRuntimeNodeIdentity.resolve(
                includeDeviceIdentity: false,
                clientID: "openclaw-ios",
                deviceID: identity.deviceId
            ) == "openclaw-ios"
        )
    }

    @Test func connectFrameEncodeCapsAndScopesAsArrays() throws {
        let data = try GatewayConnectFrameEncoder.encode(
            requestID: "req-1",
            nonce: "nonce-1",
            options: GatewayConnectOptions(
                role: "operator",
                scopes: ["operator.read", "operator.write"],
                caps: ["tool-events"],
                commands: [],
                permissions: [:],
                clientId: "openclaw-ios",
                clientMode: "ui",
                clientDisplayName: "VisionOS Copilot",
                includeDeviceIdentity: false
            ),
            appVersion: "dev",
            identity: GatewayConnectIdentitySnapshot(
                platform: "visionOS 2.0.0",
                displayName: "VisionOS Copilot",
                instanceId: "instance-1",
                deviceFamily: "Vision",
                modelIdentifier: nil,
                locale: "en-US",
                userAgent: "visionOS test"
            ),
            deviceIdentity: nil,
            authToken: "token-123",
            authBootstrapToken: nil,
            authPassword: nil,
            signatureToken: "token-123"
        )

        let json = try JSONSerialization.jsonObject(with: data) as? [String: Any]
        let params = json?["params"] as? [String: Any]

        #expect(params?["caps"] as? [String] == ["tool-events"])
        #expect(params?["scopes"] as? [String] == ["operator.read", "operator.write"])
        #expect(params?["locale"] as? String == "en-US")
        #expect(params?["userAgent"] as? String == "visionOS test")
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
        #expect(GatewayRosclawPromptOracle.cartesianCommandGuidance.contains("move_to_cartesian_goal"))
        #expect(GatewayRosclawPromptOracle.cartesianCommandGuidance.contains("cartesian_goal"))
        #expect(GatewayRosclawPromptOracle.estopCommandGuidance.contains("/rosclaw/estop"))
    }

    @Test func spatialTargetingMathMapsWorldCoordinatesIntoRobotFrame() {
        let worldPosition = SIMD3<Float>(0.8, 0.6, 1.2)
        let basePosition = SIMD3<Float>(0.3, 0.1, 0.4)

        let robotPosition = SpatialTargetingMath.robotPosition(
            fromWorldPosition: worldPosition,
            relativeToRobotBase: basePosition
        )

        let expected = SIMD3<Float>(-0.8, -0.5, 0.5)
        let epsilon: Float = 0.0001
        #expect(abs(robotPosition.x - expected.x) < epsilon)
        #expect(abs(robotPosition.y - expected.y) < epsilon)
        #expect(abs(robotPosition.z - expected.z) < epsilon)
    }

}
