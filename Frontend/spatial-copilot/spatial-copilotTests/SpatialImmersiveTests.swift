import ARKit
import Foundation
import Testing
import RosClawBridge
import simd
@testable import spatial_copilot

struct SpatialImmersiveTests {

    @Test func basePlacementTransformKeepsRobotUpAlignedToSurfaceNormal() {
        let baseWorldPoint = SIMD3<Float>(0.4, 1.1, -0.2)
        let surfaceNormal = simd_normalize(SIMD3<Float>(0.05, 0.98, -0.03))

        let transform = SpatialTargetingMath.basePlacementTransform(
            baseWorldPoint: baseWorldPoint,
            surfaceNormal: surfaceNormal
        )
        let resolvedUp = simd_normalize(
            SpatialTargetingMath.transform(direction: SIMD3<Float>(0, 0, 1), with: transform)
        )
        let basis = simd_float3x3(
            SIMD3<Float>(transform.columns.0.x, transform.columns.0.y, transform.columns.0.z),
            SIMD3<Float>(transform.columns.1.x, transform.columns.1.y, transform.columns.1.z),
            SIMD3<Float>(transform.columns.2.x, transform.columns.2.y, transform.columns.2.z)
        )

        #expect(simd_length(resolvedUp - surfaceNormal) < 0.001)
        #expect(simd_length(SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z) - baseWorldPoint) < 0.001)
        #expect(simd_determinant(basis) > 0.99)
    }

    @Test func calibrationSolveRoundTripsBetweenWorldAndRobotFrames() throws {
        let baseWorldPoint = SIMD3<Float>(0.82, 1.05, -0.34)
        let expectedRotation = simd_quatf(angle: .pi / 5, axis: simd_normalize(SIMD3<Float>(0.2, 1.0, -0.15)))
        var expectedRobotToWorld = simd_float4x4(expectedRotation)
        expectedRobotToWorld.columns.3 = SIMD4<Float>(baseWorldPoint.x, baseWorldPoint.y, baseWorldPoint.z, 1)

        let toolPose = SIMD3<Float>(0.22, -0.04, 0.19)
        let gripperWorldPoint = SpatialTargetingMath.transform(point: toolPose, with: expectedRobotToWorld)
        let surfaceNormal = simd_act(expectedRotation, SIMD3<Float>(0, 0, 1))

        let solved = try #require(
            SpatialTargetingMath.solveCalibration(
                baseWorldPoint: baseWorldPoint,
                gripperWorldPoint: gripperWorldPoint,
                toolPoseInRobotFrame: toolPose,
                surfaceNormal: surfaceNormal
            )
        )

        let resolvedBase = SpatialTargetingMath.robotPosition(
            fromWorldPosition: baseWorldPoint,
            using: solved.worldToRobotTransform
        )
        let resolvedTool = SpatialTargetingMath.robotPosition(
            fromWorldPosition: gripperWorldPoint,
            using: solved.worldToRobotTransform
        )
        let roundTrippedWorld = SpatialTargetingMath.worldPosition(
            fromRobotPosition: toolPose,
            using: solved.robotToWorldTransform
        )

        #expect(simd_length(resolvedBase) < 0.001)
        #expect(simd_length(resolvedTool - toolPose) < 0.001)
        #expect(simd_length(roundTrippedWorld - gripperWorldPoint) < 0.001)
    }

    @Test func workspaceClampingKeepsResolvedTargetInsideSafeBounds() {
        let bounds = WorkspaceBounds(xMin: -0.3, xMax: 0.5, yMin: -0.4, yMax: 0.4, zMin: 0.0, zMax: 0.5)
        let requested = SIMD3<Float>(208.0, -155.0, 43.0)
        let clamped = SpatialTargetingMath.clamp(requested, to: bounds)

        #expect(clamped.x == Float(bounds.xMax))
        #expect(clamped.y == Float(bounds.yMin))
        #expect(clamped.z == Float(bounds.zMax))
        #expect(SpatialTargetingMath.isInside(clamped, bounds: bounds))
    }

    @Test func previewStateKeepsDisplayedTargetAtRawStylusPointWhenOutsideWorkspace() {
        let bounds = WorkspaceBounds(xMin: -0.2, xMax: 0.2, yMin: -0.2, yMax: 0.2, zMin: 0.0, zMax: 0.3)
        let rawWorldPoint = SIMD3<Float>(0.42, 0.18, 0.44)

        let preview = SpatialTargetingMath.previewState(
            targetWorldPoint: rawWorldPoint,
            robotToWorldTransform: matrix_identity_float4x4,
            workspaceBounds: bounds,
            seedJointPositions: [:],
            maxIKErrorMeters: 0.03
        )

        #expect(preview.targetWorldPoint == rawWorldPoint)
        #expect(preview.requestedRobotPoint == rawWorldPoint)
        #expect(preview.clampedRobotPoint != rawWorldPoint)
        #expect(!preview.insideWorkspace)
        #expect(!preview.isValid)
        #expect(preview.invalidReason == .outsideWorkspace)
    }

    @Test func planeProjectorOnlyReturnsPointsInsidePlaneMesh() {
        let planeMesh = SpatialPlaneMesh(
            worldFromPlaneTransform: matrix_identity_float4x4,
            surfaceNormal: SIMD3<Float>(0, 1, 0),
            triangles: [
                .init(
                    a: SIMD2<Float>(-0.4, -0.4),
                    b: SIMD2<Float>(0.4, -0.4),
                    c: SIMD2<Float>(0.0, 0.4)
                )
            ]
        )

        let insidePoint = SIMD3<Float>(0.0, 0.18, 0.0)
        let insideProjection = SpatialPlaneProjector.project(
            point: insidePoint,
            onto: planeMesh,
            maxDistance: 0.25
        )
        let outsideProjection = SpatialPlaneProjector.project(
            point: SIMD3<Float>(0.45, 0.18, 0.35),
            onto: planeMesh,
            maxDistance: 0.25
        )

        #expect(insideProjection != nil)
        #expect(simd_length(insideProjection?.worldPoint ?? .zero) < 0.001)
        #expect(outsideProjection == nil)
    }

    @Test func surfaceResolverPrefersRaycastWhenItAgreesWithMeshProjection() {
        let projection = SpatialPlaneProjector.ProjectionCandidate(
            anchorID: UUID(),
            worldPoint: SIMD3<Float>(0.12, 0.88, -0.24),
            surfaceNormal: SIMD3<Float>(0, 1, 0),
            distanceToPoint: 0.02,
            surfaceArea: 0.7,
            classificationBias: 0.0
        )
        let raycast = SpatialPlaneProjector.RaycastCandidate(
            anchorID: projection.anchorID,
            worldPoint: projection.worldPoint + SIMD3<Float>(0.008, 0, 0.002),
            surfaceNormal: projection.surfaceNormal,
            distanceToPoint: 0.021,
            surfaceArea: 0.7,
            classificationBias: 0.0
        )

        let resolved = SpatialPlaneProjector.resolvePreferredCandidate(
            raycastCandidate: raycast,
            projectionCandidates: [projection]
        )

        #expect(resolved?.method == .raycast)
        #expect((resolved?.comparisonDeltaMeters ?? 1) < 0.05)
    }

    @Test func surfaceResolverFallsBackToMeshProjectionWhenRaycastDisagrees() {
        let anchorID = UUID()
        let projection = SpatialPlaneProjector.ProjectionCandidate(
            anchorID: anchorID,
            worldPoint: SIMD3<Float>(0.0, 0.9, 0.0),
            surfaceNormal: SIMD3<Float>(0, 1, 0),
            distanceToPoint: 0.01,
            surfaceArea: 0.9,
            classificationBias: 0.0
        )
        let raycast = SpatialPlaneProjector.RaycastCandidate(
            anchorID: UUID(),
            worldPoint: SIMD3<Float>(0.18, 0.9, 0.14),
            surfaceNormal: SIMD3<Float>(0, 1, 0),
            distanceToPoint: 0.19,
            surfaceArea: 0.3,
            classificationBias: 0.12
        )

        let resolved = SpatialPlaneProjector.resolvePreferredCandidate(
            raycastCandidate: raycast,
            projectionCandidates: [projection]
        )

        #expect(resolved?.method == .meshProjection)
        #expect(resolved?.anchorID == anchorID)
        #expect((resolved?.comparisonDeltaMeters ?? 0) > 0.05)
    }

    @Test func previewIKConvergesForReachableTargetAndPreservesWristRoll() {
        let targetAngles = SO101JointAngles(
            shoulderPan: 0.28,
            shoulderLift: -1.02,
            elbowFlex: 1.56,
            wristFlex: -0.48,
            wristRoll: 0.61
        )
        let target = SO101Kinematics.endEffectorPosition(jointAngles: targetAngles)
        let seed = SO101JointAngles(
            shoulderPan: 0.0,
            shoulderLift: -0.7,
            elbowFlex: 1.0,
            wristFlex: -0.3,
            wristRoll: 0.61
        )

        let result = SO101PreviewIK().solve(
            targetInBaseFrame: target,
            seededFrom: seed,
            preserveWristRoll: true
        )

        #expect(result.positionErrorMeters < 0.02)
        #expect(abs(result.jointAngles.wristRoll - seed.wristRoll) < 0.0001)
    }

    @Test func usdAuthoredNeutralBodyTransformsDriveAllSixVisualBodies() throws {
        let forwardKinematics = SO101Kinematics.forwardKinematics(
            jointAngles: .init(),
            gripperAngle: 0.0
        )
        let expectedTranslations: [String: SIMD3<Double>] = [
            "base": SIMD3<Double>(0.0, 0.0, 0.0),
            "shoulder": SIMD3<Double>(0.02079089917242527, -0.023074500262737274, 0.09488169848918915),
            "upper_arm": SIMD3<Double>(0.0025133490562438965, -0.05347376689314842, 0.149081751704216),
            "lower_arm": SIMD3<Double>(0.00251384056173265, -0.08147336542606354, 0.261651873588562),
            "wrist": SIMD3<Double>(0.002514801686629653, -0.21637339890003204, 0.26685187220573425),
            "gripper": SIMD3<Double>(0.02061525173485279, -0.2774733006954193, 0.2668520510196686),
            "jaw": SIMD3<Double>(0.03941557556390762, -0.3008729815483093, 0.2870521545410156),
        ]
        let expectedOrientations: [String: simd_quatd] = [
            "base": simd_quatd(ix: 0.0, iy: 0.0, iz: 0.0, r: 1.0),
            "shoulder": simd_quatd(ix: -0.7071055173873901, iy: -0.7071080803871155, iz: 8.963237405623659e-7, r: 8.963205004874908e-7),
            "upper_arm": simd_quatd(ix: 0.000003845626451948192, iy: -0.7071077823638916, iz: -0.0000012814998626708984, r: -0.7071058750152588),
            "lower_arm": simd_quatd(ix: -0.49999889731407166, iy: -0.5000025629997253, iz: -0.5000011920928955, r: -0.4999975860118866),
            "wrist": simd_quatd(ix: 0.000003874301910400391, iy: -0.7071078419685364, iz: -0.0000012516975402832031, r: -0.7071059346199036),
            "gripper": simd_quatd(ix: -0.49999964237213135, iy: -0.5000036358833313, iz: -0.4999951124191284, r: 0.5000019669532776),
            "jaw": simd_quatd(ix: 0.0000029206275939941406, iy: -0.7071058750152588, iz: 0.0000073015689849853516, r: 0.7071079611778259),
        ]

        for (body, expectedTranslation) in expectedTranslations {
            let transform = try #require(forwardKinematics.bodyModelTransforms[body])
            #expect(simd_length(transform.translation - expectedTranslation) < 0.000001)
        }
        for (body, expectedOrientation) in expectedOrientations {
            let transform = try #require(forwardKinematics.bodyModelTransforms[body])
            let actualOrientation = simd_quatd(transform.upperLeft3x3)
            #expect(quaternionsMatch(actualOrientation, expectedOrientation))
        }
    }

    @Test func previewIKRespectsJointLimitsForUnreachableTarget() {
        let result = SO101PreviewIK().solve(
            targetInBaseFrame: SIMD3<Double>(1.8, 1.3, 1.6),
            seededFrom: .init()
        )

        for joint in SO101ArmJoint.allCases {
            let angle = result.jointAngles[joint]
            let limits = SO101Kinematics.metadata(for: joint).limits
            #expect(angle >= limits.lowerBound)
            #expect(angle <= limits.upperBound)
        }
        #expect(result.positionErrorMeters > 0.05)
    }

    @Test func jointStreamingPolicyRequiresMeaningfulDeltaAndLowIkError() {
        let policy = SpatialJointStreamingPolicy()
        let preview = SpatialSessionModel.SpatialPreviewState(
            targetWorldPoint: SIMD3<Float>(0.4, 1.1, -0.2),
            requestedRobotPoint: SIMD3<Float>(0.20, 0.01, 0.22),
            clampedRobotPoint: SIMD3<Float>(0.20, 0.01, 0.22),
            insideWorkspace: true,
            previewJointPositions: [
                "shoulder_pan": 0.10,
                "shoulder_lift": -0.9,
                "elbow_flex": 1.4,
                "wrist_flex": -0.5,
                "wrist_roll": 0.2,
            ],
            previewEndEffectorWorldPoint: SIMD3<Float>(0.42, 1.13, -0.18),
            positionErrorMeters: 0.002,
            isValid: true,
            invalidReason: nil
        )

        #expect(policy.shouldStream(preview: preview, previous: nil, now: 1.0))

        let previous = SpatialJointStreamingState(
            jointPositions: preview.previewJointPositions,
            endEffectorWorldPoint: preview.previewEndEffectorWorldPoint,
            sentAt: 1.0
        )
        #expect(!policy.shouldStream(preview: preview, previous: previous, now: 1.01))

        let movedPreview = SpatialSessionModel.SpatialPreviewState(
            targetWorldPoint: preview.targetWorldPoint,
            requestedRobotPoint: preview.requestedRobotPoint,
            clampedRobotPoint: preview.clampedRobotPoint,
            insideWorkspace: true,
            previewJointPositions: preview.previewJointPositions.merging(["wrist_flex": -0.55]) { _, new in new },
            previewEndEffectorWorldPoint: preview.previewEndEffectorWorldPoint + SIMD3<Float>(0.01, 0, 0),
            positionErrorMeters: 0.002,
            isValid: true,
            invalidReason: nil
        )
        #expect(policy.shouldStream(preview: movedPreview, previous: previous, now: 1.2))

        let badPreview = SpatialSessionModel.SpatialPreviewState(
            targetWorldPoint: preview.targetWorldPoint,
            requestedRobotPoint: preview.requestedRobotPoint,
            clampedRobotPoint: preview.clampedRobotPoint,
            insideWorkspace: true,
            previewJointPositions: preview.previewJointPositions,
            previewEndEffectorWorldPoint: preview.previewEndEffectorWorldPoint,
            positionErrorMeters: 0.08,
            isValid: false,
            invalidReason: .ikError
        )
        #expect(!policy.canStream(preview: badPreview))

        let outsideWorkspacePreview = SpatialSessionModel.SpatialPreviewState(
            targetWorldPoint: preview.targetWorldPoint + SIMD3<Float>(0.3, 0.0, 0.0),
            requestedRobotPoint: preview.requestedRobotPoint + SIMD3<Float>(0.3, 0.0, 0.0),
            clampedRobotPoint: preview.clampedRobotPoint,
            insideWorkspace: false,
            previewJointPositions: preview.previewJointPositions,
            previewEndEffectorWorldPoint: preview.previewEndEffectorWorldPoint,
            positionErrorMeters: 0.002,
            isValid: false,
            invalidReason: .outsideWorkspace
        )
        #expect(!policy.canStream(preview: outsideWorkspacePreview))
    }

    @Test func packagedVisualAlignmentMapsUsdForwardIntoKinematicForward() {
        let correctedForward = simd_normalize(
            SpatialTargetingMath.transform(
                direction: SIMD3<Float>(0, -1, 0),
                with: SO101VisualAlignment.kinematicBaseFromModelTransform
            )
        )
        let correctedUp = simd_normalize(
            SpatialTargetingMath.transform(
                direction: SIMD3<Float>(0, 0, 1),
                with: SO101VisualAlignment.kinematicBaseFromModelTransform
            )
        )

        #expect(simd_length(correctedForward - SIMD3<Float>(1, 0, 0)) < 0.001)
        #expect(simd_length(correctedUp - SIMD3<Float>(0, 0, 1)) < 0.001)
    }

    @MainActor
    @Test func spatialSessionTransitionsBetweenRealToSimAndSimToRealHolding() throws {
        let session = SpatialSessionModel()
        let pendingTransform = SpatialTargetingMath.basePlacementTransform(
            baseWorldPoint: SIMD3<Float>(0.5, 1.0, -0.2),
            surfaceNormal: SIMD3<Float>(0, 1, 0)
        )
        session.registerBasePlacement(
            surfaceAnchorID: UUID(),
            surfaceNormal: SIMD3<Float>(0, 1, 0),
            worldPoint: SIMD3<Float>(0.5, 1.0, -0.2),
            robotToWorldTransform: pendingTransform
        )
        #expect(session.onboardingStage == .placingGripper)

        let finalTransform = SpatialTargetingMath.basePlacementTransform(
            baseWorldPoint: SIMD3<Float>(0.5, 1.0, -0.2),
            surfaceNormal: SIMD3<Float>(0, 1, 0),
            forwardHint: SIMD3<Float>(1, 0, 0)
        )
        #expect(session.completeCalibration(
            worldAnchorID: nil,
            gripperWorldPoint: SIMD3<Float>(0.62, 1.18, -0.16),
            robotToWorldTransform: finalTransform
        ))
        #expect(session.onboardingStage == .previewing)
        #expect(session.controlMode == .realToSim)

        session.beginSimToRealHolding()
        #expect(session.controlMode == .simToRealHolding)

        session.endSimToRealHolding()
        #expect(session.controlMode == .realToSim)
    }

    @MainActor
    @Test func packagedRobotAssetLoadsAndFindsExpectedBodyBindings() async {
        let model = await RobotModelEntity()
        #expect(model.didLoadPackagedAsset)
        #expect(model.hasExpectedBodyBindings)
    }
}

private func quaternionsMatch(
    _ lhs: simd_quatd,
    _ rhs: simd_quatd,
    tolerance: Double = 0.000001
) -> Bool {
    abs(simd_dot(lhs.vector, rhs.vector)) >= (1.0 - tolerance)
}
