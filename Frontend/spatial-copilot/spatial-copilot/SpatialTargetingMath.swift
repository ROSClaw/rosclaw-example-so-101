import Foundation
import RosClawBridge
import simd

struct SpatialJointStreamingState {
    let jointPositions: [String: Double]
    let endEffectorWorldPoint: SIMD3<Float>
    let sentAt: TimeInterval
}

struct SpatialJointStreamingPolicy {
    var minimumSendInterval: TimeInterval = 1.0 / 15.0
    var jointDeadbandRadians: Double = 0.01
    var endEffectorDeadbandMeters: Float = 0.005
    var maxIKErrorMeters: Double = 0.03

    func canStream(preview: SpatialSessionModel.SpatialPreviewState) -> Bool {
        preview.isValid && preview.positionErrorMeters <= maxIKErrorMeters
    }

    func shouldStream(
        preview: SpatialSessionModel.SpatialPreviewState,
        previous: SpatialJointStreamingState?,
        now: TimeInterval
    ) -> Bool {
        guard canStream(preview: preview) else { return false }
        guard let previous else { return true }
        guard (now - previous.sentAt) >= minimumSendInterval else { return false }

        if simd_length(preview.previewEndEffectorWorldPoint - previous.endEffectorWorldPoint) >= endEffectorDeadbandMeters {
            return true
        }

        for jointName in followerJointNames {
            let next = preview.previewJointPositions[jointName] ?? 0.0
            let prior = previous.jointPositions[jointName] ?? 0.0
            if abs(next - prior) >= jointDeadbandRadians {
                return true
            }
        }

        return false
    }
}

enum SpatialTargetingMath {
    struct CalibrationSolution {
        let robotToWorldTransform: simd_float4x4
        let worldToRobotTransform: simd_float4x4
    }

    static func solveCalibration(
        baseWorldPoint: SIMD3<Float>,
        gripperWorldPoint: SIMD3<Float>,
        toolPoseInRobotFrame: SIMD3<Float>,
        surfaceNormal: SIMD3<Float>
    ) -> CalibrationSolution? {
        let robotToolDirection = normalized(toolPoseInRobotFrame)
        let worldToolDirection = normalized(gripperWorldPoint - baseWorldPoint)
        guard simd_length_squared(robotToolDirection) > 0.000001,
              simd_length_squared(worldToolDirection) > 0.000001 else {
            return nil
        }

        let up = normalized(simd_dot(surfaceNormal, SIMD3<Float>(0, 1, 0)) >= 0 ? surfaceNormal : -surfaceNormal)
        let initialAlignment = shortestRotation(from: robotToolDirection, to: worldToolDirection)

        let rotatedUp = simd_act(initialAlignment, SIMD3<Float>(0, 0, 1))
        let upProjection = projectOntoPlane(rotatedUp, normal: worldToolDirection)
        let worldUpProjection = projectOntoPlane(up, normal: worldToolDirection)

        let finalRotation: simd_quatf
        if simd_length_squared(upProjection) > 0.000001,
           simd_length_squared(worldUpProjection) > 0.000001 {
            let twistAngle = signedAngle(
                from: normalized(upProjection),
                to: normalized(worldUpProjection),
                around: worldToolDirection
            )
            finalRotation = simd_normalize(simd_quatf(angle: twistAngle, axis: worldToolDirection) * initialAlignment)
        } else {
            finalRotation = initialAlignment
        }

        let robotToWorldTransform = simd_float4x4(rotation: finalRotation, translation: baseWorldPoint)
        return CalibrationSolution(
            robotToWorldTransform: robotToWorldTransform,
            worldToRobotTransform: robotToWorldTransform.inverse
        )
    }

    static func basePlacementTransform(
        baseWorldPoint: SIMD3<Float>,
        surfaceNormal: SIMD3<Float>,
        forwardHint: SIMD3<Float>? = nil
    ) -> simd_float4x4 {
        let up = normalized(simd_dot(surfaceNormal, SIMD3<Float>(0, 1, 0)) >= 0 ? surfaceNormal : -surfaceNormal)
        let referenceForward = forwardHint ?? SIMD3<Float>(0, -1, 0)
        let projectedForward = projectOntoPlane(referenceForward, normal: up)
        let forward: SIMD3<Float>
        if simd_length_squared(projectedForward) > 0.000001 {
            forward = normalized(projectedForward)
        } else {
            let fallbackAxis = abs(simd_dot(up, SIMD3<Float>(1, 0, 0))) < 0.9 ? SIMD3<Float>(1, 0, 0) : SIMD3<Float>(0, 1, 0)
            forward = normalized(projectOntoPlane(fallbackAxis, normal: up))
        }
        let right = normalized(simd_cross(up, forward))
        let orthonormalForward = normalized(simd_cross(right, up))

        return simd_float4x4(
            columns: (
                SIMD4<Float>(orthonormalForward.x, orthonormalForward.y, orthonormalForward.z, 0),
                SIMD4<Float>(right.x, right.y, right.z, 0),
                SIMD4<Float>(up.x, up.y, up.z, 0),
                SIMD4<Float>(baseWorldPoint.x, baseWorldPoint.y, baseWorldPoint.z, 1)
            )
        )
    }

    static func robotPosition(
        fromWorldPosition worldPosition: SIMD3<Float>,
        using worldToRobotTransform: simd_float4x4
    ) -> SIMD3<Float> {
        transform(point: worldPosition, with: worldToRobotTransform)
    }

    static func worldPosition(
        fromRobotPosition robotPosition: SIMD3<Float>,
        using robotToWorldTransform: simd_float4x4
    ) -> SIMD3<Float> {
        transform(point: robotPosition, with: robotToWorldTransform)
    }

    static func previewState(
        targetWorldPoint: SIMD3<Float>,
        robotToWorldTransform: simd_float4x4,
        workspaceBounds: WorkspaceBounds,
        seedJointPositions: [String: Double],
        maxIKErrorMeters: Double
    ) -> SpatialSessionModel.SpatialPreviewState {
        let worldToRobotTransform = robotToWorldTransform.inverse
        let requestedRobotPoint = robotPosition(
            fromWorldPosition: targetWorldPoint,
            using: worldToRobotTransform
        )
        let insideWorkspace = isInside(requestedRobotPoint, bounds: workspaceBounds)
        let clampedRobotPoint = clamp(requestedRobotPoint, to: workspaceBounds)
        let ikTarget = insideWorkspace ? requestedRobotPoint : clampedRobotPoint
        let ikResult = SO101PreviewIK().solve(
            targetInBaseFrame: SIMD3<Double>(
                Double(ikTarget.x),
                Double(ikTarget.y),
                Double(ikTarget.z)
            ),
            seededFromJointPositions: seedJointPositions,
            preserveWristRoll: true
        )
        let previewEndEffectorWorldPoint = worldPosition(
            fromRobotPosition: SIMD3<Float>(
                Float(ikResult.endEffectorPosition.x),
                Float(ikResult.endEffectorPosition.y),
                Float(ikResult.endEffectorPosition.z)
            ),
            using: robotToWorldTransform
        )

        let invalidReason: SpatialSessionModel.SpatialTargetInvalidReason?
        if !insideWorkspace {
            invalidReason = .outsideWorkspace
        } else if ikResult.positionErrorMeters > maxIKErrorMeters {
            invalidReason = .ikError
        } else {
            invalidReason = nil
        }

        return SpatialSessionModel.SpatialPreviewState(
            targetWorldPoint: targetWorldPoint,
            requestedRobotPoint: requestedRobotPoint,
            clampedRobotPoint: clampedRobotPoint,
            insideWorkspace: insideWorkspace,
            previewJointPositions: ikResult.jointAngles.asJointPositionMap,
            previewEndEffectorWorldPoint: previewEndEffectorWorldPoint,
            positionErrorMeters: ikResult.positionErrorMeters,
            isValid: invalidReason == nil,
            invalidReason: invalidReason
        )
    }

    static func clamp(_ position: SIMD3<Float>, to bounds: WorkspaceBounds) -> SIMD3<Float> {
        SIMD3<Float>(
            x: position.x.clamped(to: Float(bounds.xMin)...Float(bounds.xMax)),
            y: position.y.clamped(to: Float(bounds.yMin)...Float(bounds.yMax)),
            z: position.z.clamped(to: Float(bounds.zMin)...Float(bounds.zMax))
        )
    }

    static func isInside(_ position: SIMD3<Float>, bounds: WorkspaceBounds) -> Bool {
        (Float(bounds.xMin)...Float(bounds.xMax)).contains(position.x) &&
            (Float(bounds.yMin)...Float(bounds.yMax)).contains(position.y) &&
            (Float(bounds.zMin)...Float(bounds.zMax)).contains(position.z)
    }

    static func workspaceCorners(for bounds: WorkspaceBounds) -> [SIMD3<Float>] {
        let xs = [Float(bounds.xMin), Float(bounds.xMax)]
        let ys = [Float(bounds.yMin), Float(bounds.yMax)]
        let zs = [Float(bounds.zMin), Float(bounds.zMax)]
        return xs.flatMap { x in
            ys.flatMap { y in
                zs.map { z in SIMD3<Float>(x, y, z) }
            }
        }
    }

    static func transform(point: SIMD3<Float>, with matrix: simd_float4x4) -> SIMD3<Float> {
        let transformed = matrix * SIMD4<Float>(point.x, point.y, point.z, 1)
        return SIMD3<Float>(transformed.x, transformed.y, transformed.z)
    }

    static func transform(direction: SIMD3<Float>, with matrix: simd_float4x4) -> SIMD3<Float> {
        let transformed = matrix * SIMD4<Float>(direction.x, direction.y, direction.z, 0)
        return SIMD3<Float>(transformed.x, transformed.y, transformed.z)
    }
}

private extension simd_float4x4 {
    init(rotation: simd_quatf, translation: SIMD3<Float>) {
        self = simd_float4x4(rotation)
        columns.3 = SIMD4<Float>(translation.x, translation.y, translation.z, 1)
    }
}

private extension Float {
    func clamped(to range: ClosedRange<Float>) -> Float {
        min(max(self, range.lowerBound), range.upperBound)
    }
}

private func normalized(_ vector: SIMD3<Float>) -> SIMD3<Float> {
    let lengthSquared = simd_length_squared(vector)
    guard lengthSquared > 0.000001 else { return .zero }
    return simd_normalize(vector)
}

private func projectOntoPlane(_ vector: SIMD3<Float>, normal: SIMD3<Float>) -> SIMD3<Float> {
    vector - normal * simd_dot(vector, normal)
}

private func shortestRotation(from source: SIMD3<Float>, to target: SIMD3<Float>) -> simd_quatf {
    let dotProduct = simd_dot(source, target)
    if dotProduct <= -0.9999 {
        let fallbackAxis = normalized(abs(source.x) < 0.9 ? simd_cross(source, SIMD3<Float>(1, 0, 0)) : simd_cross(source, SIMD3<Float>(0, 1, 0)))
        return simd_quatf(angle: .pi, axis: fallbackAxis)
    }
    if dotProduct >= 0.9999 {
        return simd_quatf(angle: 0, axis: SIMD3<Float>(0, 0, 1))
    }
    return simd_quatf(from: source, to: target)
}

private func signedAngle(from source: SIMD3<Float>, to target: SIMD3<Float>, around axis: SIMD3<Float>) -> Float {
    let cross = simd_cross(source, target)
    return atan2(simd_dot(axis, cross), simd_dot(source, target))
}
