import Foundation
import Testing
import RosClawBridge
import simd
@testable import spatial_copilot

struct SpatialImmersiveTests {

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

    @MainActor
    @Test func packagedRobotAssetLoadsAndFindsExpectedBodyBindings() async {
        let model = await RobotModelEntity()
        #expect(model.didLoadPackagedAsset)
        #expect(model.hasExpectedBodyBindings)
    }
}
