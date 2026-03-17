import Foundation
import simd

internal struct SO101PreviewIKConfiguration {
    var maxIterations: Int = 28
    var positionToleranceMeters: Double = 0.003
    var stepScale: Double = 0.70
    var epsilon: Double = 1e-8
}

internal struct SO101PreviewIKResult {
    let jointAngles: SO101JointAngles
    let endEffectorPosition: SIMD3<Double>
    let positionErrorMeters: Double
    let iterations: Int
    let converged: Bool
}

/// Local constrained CCD solver for fast in-app arm preview.
///
/// v1 policy:
/// - Position-only solve in robot base frame.
/// - Wrist roll is preserved by default.
internal struct SO101PreviewIK {
    var configuration = SO101PreviewIKConfiguration()

    func solve(
        targetInBaseFrame: SIMD3<Double>,
        seededFrom seedAngles: SO101JointAngles,
        preserveWristRoll: Bool = true,
        baseTransform: simd_double4x4 = matrix_identity_double4x4
    ) -> SO101PreviewIKResult {
        var angles = SO101Kinematics.clampToJointLimits(seedAngles)
        let fixedWristRoll = angles.wristRoll
        let solveOrder: [SO101ArmJoint] = preserveWristRoll
            ? [.wristFlex, .elbowFlex, .shoulderLift, .shoulderPan]
            : [.wristRoll, .wristFlex, .elbowFlex, .shoulderLift, .shoulderPan]

        var iterationCount = 0

        for iteration in 1...max(configuration.maxIterations, 1) {
            iterationCount = iteration
            let fk = SO101Kinematics.forwardKinematics(jointAngles: angles, baseTransform: baseTransform)
            let currentError = simd_length(targetInBaseFrame - fk.endEffectorPosition)
            if currentError <= configuration.positionToleranceMeters {
                return SO101PreviewIKResult(
                    jointAngles: angles,
                    endEffectorPosition: fk.endEffectorPosition,
                    positionErrorMeters: currentError,
                    iterations: iteration,
                    converged: true
                )
            }

            for joint in solveOrder {
                let stepFK = SO101Kinematics.forwardKinematics(jointAngles: angles, baseTransform: baseTransform)
                guard let jointTransform = stepFK.jointWorldTransforms[joint] else { continue }
                let metadata = SO101Kinematics.metadata(for: joint)

                let jointWorldPosition = jointTransform.translation
                let toolWorldPosition = stepFK.endEffectorPosition
                let toTool = toolWorldPosition - jointWorldPosition
                let toTarget = targetInBaseFrame - jointWorldPosition

                if simd_length_squared(toTool) <= configuration.epsilon || simd_length_squared(toTarget) <= configuration.epsilon {
                    continue
                }

                let axisWorld = normalizedAxisWorld(
                    jointTransform: jointTransform,
                    axisInJointFrame: metadata.axisInJointFrame
                )
                let toolProjected = toTool - axisWorld * simd_dot(toTool, axisWorld)
                let targetProjected = toTarget - axisWorld * simd_dot(toTarget, axisWorld)

                let toolLength = simd_length(toolProjected)
                let targetLength = simd_length(targetProjected)
                if toolLength <= configuration.epsilon || targetLength <= configuration.epsilon {
                    continue
                }

                let toolDir = toolProjected / toolLength
                let targetDir = targetProjected / targetLength
                let cosTerm = clamp(simd_dot(toolDir, targetDir), min: -1.0, max: 1.0)
                let sinTerm = simd_dot(axisWorld, simd_cross(toolDir, targetDir))
                let signedDelta = atan2(sinTerm, cosTerm) * configuration.stepScale

                if abs(signedDelta) <= configuration.epsilon {
                    continue
                }

                let proposed = angles[joint] + signedDelta
                angles[joint] = clamp(proposed, min: metadata.limits.lowerBound, max: metadata.limits.upperBound)
                if preserveWristRoll {
                    angles.wristRoll = fixedWristRoll
                }
            }
        }

        let finalFK = SO101Kinematics.forwardKinematics(jointAngles: angles, baseTransform: baseTransform)
        let finalError = simd_length(targetInBaseFrame - finalFK.endEffectorPosition)
        return SO101PreviewIKResult(
            jointAngles: angles,
            endEffectorPosition: finalFK.endEffectorPosition,
            positionErrorMeters: finalError,
            iterations: iterationCount,
            converged: finalError <= configuration.positionToleranceMeters
        )
    }

    func solve(
        targetInBaseFrame: SIMD3<Double>,
        seededFromJointPositions jointPositions: [String: Double],
        preserveWristRoll: Bool = true,
        baseTransform: simd_double4x4 = matrix_identity_double4x4
    ) -> SO101PreviewIKResult {
        let seed = SO101Kinematics.clampToJointLimits(SO101JointAngles.seeded(from: jointPositions))
        return solve(
            targetInBaseFrame: targetInBaseFrame,
            seededFrom: seed,
            preserveWristRoll: preserveWristRoll,
            baseTransform: baseTransform
        )
    }
}

private func normalizedAxisWorld(
    jointTransform: simd_double4x4,
    axisInJointFrame: SIMD3<Double>
) -> SIMD3<Double> {
    let rotation = jointTransform.upperLeft3x3
    let axisWorld = rotation * axisInJointFrame
    return simd_length_squared(axisWorld) > 0.0 ? simd_normalize(axisWorld) : SIMD3<Double>(0.0, 0.0, 1.0)
}

private func clamp(_ value: Double, min minValue: Double, max maxValue: Double) -> Double {
    Swift.min(Swift.max(value, minValue), maxValue)
}
