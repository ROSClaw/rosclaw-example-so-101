import Foundation
import simd

/// Canonical controllable arm joints for the SO-101 follower.
internal enum SO101ArmJoint: String, CaseIterable {
    case shoulderPan = "shoulder_pan"
    case shoulderLift = "shoulder_lift"
    case elbowFlex = "elbow_flex"
    case wristFlex = "wrist_flex"
    case wristRoll = "wrist_roll"
}

internal struct SO101JointAngles: Equatable {
    var shoulderPan: Double
    var shoulderLift: Double
    var elbowFlex: Double
    var wristFlex: Double
    var wristRoll: Double

    init(
        shoulderPan: Double = 0.0,
        shoulderLift: Double = 0.0,
        elbowFlex: Double = 0.0,
        wristFlex: Double = 0.0,
        wristRoll: Double = 0.0
    ) {
        self.shoulderPan = shoulderPan
        self.shoulderLift = shoulderLift
        self.elbowFlex = elbowFlex
        self.wristFlex = wristFlex
        self.wristRoll = wristRoll
    }

    subscript(_ joint: SO101ArmJoint) -> Double {
        get {
            switch joint {
            case .shoulderPan: return shoulderPan
            case .shoulderLift: return shoulderLift
            case .elbowFlex: return elbowFlex
            case .wristFlex: return wristFlex
            case .wristRoll: return wristRoll
            }
        }
        set {
            switch joint {
            case .shoulderPan: shoulderPan = newValue
            case .shoulderLift: shoulderLift = newValue
            case .elbowFlex: elbowFlex = newValue
            case .wristFlex: wristFlex = newValue
            case .wristRoll: wristRoll = newValue
            }
        }
    }

    static func seeded(from jointPositions: [String: Double]) -> SO101JointAngles {
        SO101JointAngles(
            shoulderPan: jointPositions[SO101ArmJoint.shoulderPan.rawValue] ?? 0.0,
            shoulderLift: jointPositions[SO101ArmJoint.shoulderLift.rawValue] ?? 0.0,
            elbowFlex: jointPositions[SO101ArmJoint.elbowFlex.rawValue] ?? 0.0,
            wristFlex: jointPositions[SO101ArmJoint.wristFlex.rawValue] ?? 0.0,
            wristRoll: jointPositions[SO101ArmJoint.wristRoll.rawValue] ?? 0.0
        )
    }

    var asJointPositionMap: [String: Double] {
        [
            SO101ArmJoint.shoulderPan.rawValue: shoulderPan,
            SO101ArmJoint.shoulderLift.rawValue: shoulderLift,
            SO101ArmJoint.elbowFlex.rawValue: elbowFlex,
            SO101ArmJoint.wristFlex.rawValue: wristFlex,
            SO101ArmJoint.wristRoll.rawValue: wristRoll,
        ]
    }
}

internal enum SO101VisualAlignment {
    /// The packaged USD is authored with forward along `-Y`, while the app uses
    /// `+X` as robot-forward and `+Z` as up in the base frame.
    static let kinematicBaseFromModelTransform: simd_float4x4 = {
        var matrix = simd_float4x4(
            simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(0, 0, 1))
        )
        matrix.columns.3 = SIMD4<Float>(0, 0, 0, 1)
        return matrix
    }()
}

internal struct SO101JointMetadata {
    let joint: SO101ArmJoint
    let parentBodyName: String
    let childBodyName: String
    let originFromParent: SIMD3<Double>
    let jointFrameOrientationFromParent: simd_quatd
    let axisInJointFrame: SIMD3<Double>
    let limits: ClosedRange<Double>
}

internal struct SO101ForwardKinematics {
    let jointWorldTransforms: [SO101ArmJoint: simd_double4x4]
    let bodyModelTransforms: [String: simd_double4x4]
    let bodyWorldTransforms: [String: simd_double4x4]
    let endEffectorTransform: simd_double4x4

    var endEffectorPosition: SIMD3<Double> {
        endEffectorTransform.translation
    }
}

/// Checked-in kinematics metadata for the packaged `so101_follower.usd` hierarchy.
///
/// The visual bodies in the USD are authored as siblings under the root, while
/// the articulation itself is a fixed-base chain:
/// base -> shoulder -> upper_arm -> lower_arm -> wrist -> gripper -> jaw
///
/// These joint pivots are copied from the USD so the local IK solver and the
/// RealityKit visual pose share the exact same authored frames.
internal enum SO101Kinematics {

    static let bodyHierarchy: [String] = [
        "base",
        "shoulder",
        "upper_arm",
        "lower_arm",
        "wrist",
        "gripper",
        "jaw",
    ]

    static let jointChain: [SO101JointMetadata] = [
        SO101JointMetadata(
            joint: .shoulderPan,
            parentBodyName: "base",
            childBodyName: "shoulder",
            originFromParent: SIMD3<Double>(0.0207909, -0.0230745, 0.0948817),
            jointFrameOrientationFromParent: usdQuaternion(
                r: 8.963205e-7,
                x: -0.7071055,
                y: -0.7071081,
                z: 8.9632374e-7
            ),
            axisInJointFrame: SIMD3<Double>(0.0, 0.0, 1.0),
            limits: radians(-109.99987...109.99987)
        ),
        SO101JointMetadata(
            joint: .shoulderLift,
            parentBodyName: "shoulder",
            childBodyName: "upper_arm",
            originFromParent: SIMD3<Double>(-0.0303992, -0.0182778, -0.0542),
            jointFrameOrientationFromParent: usdQuaternion(
                r: 0.4999982,
                x: -0.5,
                y: -0.5,
                z: -0.50000185
            ),
            axisInJointFrame: SIMD3<Double>(0.0, 0.0, 1.0),
            limits: radians(-100.00004...100.00004)
        ),
        SO101JointMetadata(
            joint: .elbowFlex,
            parentBodyName: "upper_arm",
            childBodyName: "lower_arm",
            originFromParent: SIMD3<Double>(-0.11257, -0.028, 0.0),
            jointFrameOrientationFromParent: usdQuaternion(
                r: 0.7071055,
                x: -6.378481e-16,
                y: -2.3060708e-16,
                z: 0.7071081
            ),
            axisInJointFrame: SIMD3<Double>(0.0, 0.0, 1.0),
            limits: radians(-100.00004...90.000206)
        ),
        SO101JointMetadata(
            joint: .wristFlex,
            parentBodyName: "lower_arm",
            childBodyName: "wrist",
            originFromParent: SIMD3<Double>(-0.1349, 0.0052, 0.0),
            jointFrameOrientationFromParent: usdQuaternion(
                r: 0.7071055,
                x: 2.1600662e-15,
                y: -1.3619623e-16,
                z: -0.7071081
            ),
            axisInJointFrame: SIMD3<Double>(0.0, 0.0, 1.0),
            limits: radians(-94.99983...94.99983)
        ),
        SO101JointMetadata(
            joint: .wristRoll,
            parentBodyName: "wrist",
            childBodyName: "gripper",
            originFromParent: SIMD3<Double>(0.0, -0.0611, 0.0181),
            jointFrameOrientationFromParent: usdQuaternion(
                r: 8.631542e-7,
                x: 9.2948994e-7,
                y: 0.7071081,
                z: 0.7071055
            ),
            axisInJointFrame: SIMD3<Double>(0.0, 0.0, 1.0),
            limits: radians(-160.00018...160.00018)
        ),
    ]

    /// From the kinematic gripper frame to the tool-point used for local preview.
    static let toolTipOffsetFromGripper = SIMD3<Double>(0.048, 0.0, 0.0)

    static let gripperOpenRangeMeters: ClosedRange<Double> = 0.0...0.042

    static let defaultJointAngles = SO101JointAngles(
        shoulderPan: 0.0,
        shoulderLift: -0.85,
        elbowFlex: 1.45,
        wristFlex: -0.60,
        wristRoll: 0.0
    )

    private static let jawJointOriginFromGripper = SIMD3<Double>(0.0202, 0.0188, -0.0234)
    private static let jawJointOrientationFromGripper = usdQuaternion(
        r: 0.7071055,
        x: 0.7071081,
        y: -4.92038e-15,
        z: -4.8840093e-15
    )
    private static let jawLimits = radians(-10.000004...100.00004)
    private static let kinematicBaseFromModelTransformDouble = simd_double4x4(
        SO101VisualAlignment.kinematicBaseFromModelTransform
    )

    static func metadata(for joint: SO101ArmJoint) -> SO101JointMetadata {
        guard let entry = jointChain.first(where: { $0.joint == joint }) else {
            fatalError("Missing SO-101 metadata for \(joint.rawValue)")
        }
        return entry
    }

    static func clampToJointLimits(_ angles: SO101JointAngles) -> SO101JointAngles {
        var clamped = angles
        for metadata in jointChain {
            clamped[metadata.joint] = min(max(clamped[metadata.joint], metadata.limits.lowerBound), metadata.limits.upperBound)
        }
        return clamped
    }

    static func jawAngle(forOpenFraction openFraction: Double?) -> Double {
        let fraction = min(max(openFraction ?? 0.5, 0.0), 1.0)
        let lower = jawLimits.lowerBound * 180.0 / .pi
        let upper = jawLimits.upperBound * 180.0 / .pi
        let degrees = lower + ((upper - lower) * fraction)
        return degrees * .pi / 180.0
    }

    static func forwardKinematics(
        jointAngles: SO101JointAngles,
        gripperAngle: Double = 0.0,
        baseTransform: simd_double4x4 = matrix_identity_double4x4
    ) -> SO101ForwardKinematics {
        let modelPose = modelForwardKinematics(
            jointAngles: jointAngles,
            gripperAngle: clamp(gripperAngle, min: jawLimits.lowerBound, max: jawLimits.upperBound)
        )
        let jointTransforms = modelPose.jointModelTransforms.mapValues {
            baseTransform * kinematicBaseFromModelTransformDouble * $0
        }
        let bodyTransforms = modelPose.bodyModelTransforms.mapValues {
            baseTransform * kinematicBaseFromModelTransformDouble * $0
        }
        let gripperTransform = bodyTransforms["gripper"] ?? baseTransform
        let endEffector = gripperTransform * translationMatrix(toolTipOffsetFromGripper)

        return SO101ForwardKinematics(
            jointWorldTransforms: jointTransforms,
            bodyModelTransforms: modelPose.bodyModelTransforms,
            bodyWorldTransforms: bodyTransforms,
            endEffectorTransform: endEffector
        )
    }

    static func endEffectorPosition(
        jointAngles: SO101JointAngles,
        gripperAngle: Double = 0.0,
        baseTransform: simd_double4x4 = matrix_identity_double4x4
    ) -> SIMD3<Double> {
        forwardKinematics(
            jointAngles: jointAngles,
            gripperAngle: gripperAngle,
            baseTransform: baseTransform
        ).endEffectorPosition
    }

    static func endEffectorPosition(
        jointPositions: [String: Double],
        gripperAngle: Double = 0.0,
        baseTransform: simd_double4x4 = matrix_identity_double4x4
    ) -> SIMD3<Double> {
        let seededAngles = clampToJointLimits(SO101JointAngles.seeded(from: jointPositions))
        return endEffectorPosition(
            jointAngles: seededAngles,
            gripperAngle: gripperAngle,
            baseTransform: baseTransform
        )
    }

    private static func modelForwardKinematics(
        jointAngles: SO101JointAngles,
        gripperAngle: Double
    ) -> SO101ModelForwardKinematics {
        var jointTransforms: [SO101ArmJoint: simd_double4x4] = [:]
        var bodyTransforms: [String: simd_double4x4] = ["base": matrix_identity_double4x4]

        for metadata in jointChain {
            let parentTransform = bodyTransforms[metadata.parentBodyName] ?? matrix_identity_double4x4
            let childTransform = childBodyTransform(
                parentTransform: parentTransform,
                originFromParent: metadata.originFromParent,
                jointFrameOrientationFromParent: metadata.jointFrameOrientationFromParent,
                axisInJointFrame: metadata.axisInJointFrame,
                jointAngle: jointAngles[metadata.joint]
            )
            jointTransforms[metadata.joint] = childTransform
            bodyTransforms[metadata.childBodyName] = childTransform
        }

        let jawParentTransform = bodyTransforms["gripper"] ?? matrix_identity_double4x4
        bodyTransforms["jaw"] = childBodyTransform(
            parentTransform: jawParentTransform,
            originFromParent: jawJointOriginFromGripper,
            jointFrameOrientationFromParent: jawJointOrientationFromGripper,
            axisInJointFrame: SIMD3<Double>(0.0, 0.0, 1.0),
            jointAngle: gripperAngle
        )

        return SO101ModelForwardKinematics(
            jointModelTransforms: jointTransforms,
            bodyModelTransforms: bodyTransforms
        )
    }
}

private struct SO101ModelForwardKinematics {
    let jointModelTransforms: [SO101ArmJoint: simd_double4x4]
    let bodyModelTransforms: [String: simd_double4x4]
}

internal extension simd_double4x4 {
    var translation: SIMD3<Double> {
        SIMD3<Double>(columns.3.x, columns.3.y, columns.3.z)
    }

    var upperLeft3x3: simd_double3x3 {
        simd_double3x3(
            SIMD3<Double>(columns.0.x, columns.0.y, columns.0.z),
            SIMD3<Double>(columns.1.x, columns.1.y, columns.1.z),
            SIMD3<Double>(columns.2.x, columns.2.y, columns.2.z)
        )
    }
}

private func childBodyTransform(
    parentTransform: simd_double4x4,
    originFromParent: SIMD3<Double>,
    jointFrameOrientationFromParent: simd_quatd,
    axisInJointFrame: SIMD3<Double>,
    jointAngle: Double
) -> simd_double4x4 {
    let jointFrameTransform =
        parentTransform *
        translationMatrix(originFromParent) *
        simd_double4x4(jointFrameOrientationFromParent)
    return jointFrameTransform * rotationMatrix(axis: axisInJointFrame, angle: jointAngle)
}

private func translationMatrix(_ t: SIMD3<Double>) -> simd_double4x4 {
    simd_double4x4(
        SIMD4<Double>(1.0, 0.0, 0.0, 0.0),
        SIMD4<Double>(0.0, 1.0, 0.0, 0.0),
        SIMD4<Double>(0.0, 0.0, 1.0, 0.0),
        SIMD4<Double>(t.x, t.y, t.z, 1.0)
    )
}

private func rotationMatrix(axis: SIMD3<Double>, angle: Double) -> simd_double4x4 {
    let normalizedAxis = simd_length_squared(axis) > 0.0 ? simd_normalize(axis) : SIMD3<Double>(0.0, 0.0, 1.0)
    let quaternion = simd_quatd(angle: angle, axis: normalizedAxis)
    return simd_double4x4(quaternion)
}

private func clamp(_ value: Double, min minValue: Double, max maxValue: Double) -> Double {
    Swift.min(Swift.max(value, minValue), maxValue)
}

private func radians(_ degrees: ClosedRange<Double>) -> ClosedRange<Double> {
    (degrees.lowerBound * .pi / 180.0)...(degrees.upperBound * .pi / 180.0)
}

private func usdQuaternion(r: Double, x: Double, y: Double, z: Double) -> simd_quatd {
    simd_quatd(ix: x, iy: y, iz: z, r: r)
}

private extension simd_double4x4 {
    init(_ matrix: simd_float4x4) {
        self.init(
            SIMD4<Double>(Double(matrix.columns.0.x), Double(matrix.columns.0.y), Double(matrix.columns.0.z), Double(matrix.columns.0.w)),
            SIMD4<Double>(Double(matrix.columns.1.x), Double(matrix.columns.1.y), Double(matrix.columns.1.z), Double(matrix.columns.1.w)),
            SIMD4<Double>(Double(matrix.columns.2.x), Double(matrix.columns.2.y), Double(matrix.columns.2.z), Double(matrix.columns.2.w)),
            SIMD4<Double>(Double(matrix.columns.3.x), Double(matrix.columns.3.y), Double(matrix.columns.3.z), Double(matrix.columns.3.w))
        )
    }
}
