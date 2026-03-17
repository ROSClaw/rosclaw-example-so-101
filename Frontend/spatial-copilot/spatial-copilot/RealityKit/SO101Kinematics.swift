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

internal struct SO101JointMetadata {
    let joint: SO101ArmJoint
    let parentBodyName: String
    let childBodyName: String
    let originFromParent: SIMD3<Double>
    let axisInJointFrame: SIMD3<Double>
    let childOffset: SIMD3<Double>
    let limits: ClosedRange<Double>
}

internal struct SO101ForwardKinematics {
    let jointWorldTransforms: [SO101ArmJoint: simd_double4x4]
    let bodyWorldTransforms: [String: simd_double4x4]
    let endEffectorTransform: simd_double4x4

    var endEffectorPosition: SIMD3<Double> {
        endEffectorTransform.translation
    }
}

/// Checked-in kinematics metadata for the packaged `so101_follower.usd` hierarchy.
///
/// Hierarchy reference:
/// base -> shoulder -> upper_arm -> lower_arm -> wrist -> gripper -> jaw
///
/// The link offsets are intentionally explicit and metric so the preview solver can
/// run locally without parsing USD at runtime.
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
            originFromParent: SIMD3<Double>(0.0, 0.0, 0.085),
            axisInJointFrame: SIMD3<Double>(0.0, 0.0, 1.0),
            childOffset: SIMD3<Double>(0.0, 0.0, 0.038),
            limits: (-Double.pi)...(Double.pi)
        ),
        SO101JointMetadata(
            joint: .shoulderLift,
            parentBodyName: "shoulder",
            childBodyName: "upper_arm",
            originFromParent: SIMD3<Double>(0.0, 0.0, 0.0),
            axisInJointFrame: SIMD3<Double>(0.0, 1.0, 0.0),
            childOffset: SIMD3<Double>(0.126, 0.0, 0.0),
            limits: (-2.35)...(1.20)
        ),
        SO101JointMetadata(
            joint: .elbowFlex,
            parentBodyName: "upper_arm",
            childBodyName: "lower_arm",
            originFromParent: SIMD3<Double>(0.0, 0.0, 0.0),
            axisInJointFrame: SIMD3<Double>(0.0, 1.0, 0.0),
            childOffset: SIMD3<Double>(0.118, 0.0, 0.0),
            limits: (-2.60)...(2.60)
        ),
        SO101JointMetadata(
            joint: .wristFlex,
            parentBodyName: "lower_arm",
            childBodyName: "wrist",
            originFromParent: SIMD3<Double>(0.0, 0.0, 0.0),
            axisInJointFrame: SIMD3<Double>(0.0, 1.0, 0.0),
            childOffset: SIMD3<Double>(0.073, 0.0, 0.0),
            limits: (-2.70)...(2.70)
        ),
        SO101JointMetadata(
            joint: .wristRoll,
            parentBodyName: "wrist",
            childBodyName: "gripper",
            originFromParent: SIMD3<Double>(0.0, 0.0, 0.0),
            axisInJointFrame: SIMD3<Double>(1.0, 0.0, 0.0),
            childOffset: SIMD3<Double>(0.058, 0.0, 0.0),
            limits: (-Double.pi)...(Double.pi)
        ),
    ]

    /// From the gripper body frame to the jaw/tip tracking point.
    static let toolTipOffsetFromGripper = SIMD3<Double>(0.048, 0.0, 0.0)

    static let gripperOpenRangeMeters: ClosedRange<Double> = 0.0...0.042

    static let defaultJointAngles = SO101JointAngles(
        shoulderPan: 0.0,
        shoulderLift: -0.85,
        elbowFlex: 1.45,
        wristFlex: -0.60,
        wristRoll: 0.0
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

    static func forwardKinematics(
        jointAngles: SO101JointAngles,
        baseTransform: simd_double4x4 = matrix_identity_double4x4
    ) -> SO101ForwardKinematics {
        var current = baseTransform
        var jointTransforms: [SO101ArmJoint: simd_double4x4] = [:]
        var bodyTransforms: [String: simd_double4x4] = ["base": baseTransform]

        for metadata in jointChain {
            current = current * translationMatrix(metadata.originFromParent)
            let rotation = rotationMatrix(axis: metadata.axisInJointFrame, angle: jointAngles[metadata.joint])
            let childTransform = current * rotation
            jointTransforms[metadata.joint] = childTransform
            bodyTransforms[metadata.childBodyName] = childTransform
            current = childTransform * translationMatrix(metadata.childOffset)
        }

        let endEffector = current * translationMatrix(toolTipOffsetFromGripper)
        bodyTransforms["jaw"] = endEffector

        return SO101ForwardKinematics(
            jointWorldTransforms: jointTransforms,
            bodyWorldTransforms: bodyTransforms,
            endEffectorTransform: endEffector
        )
    }

    static func endEffectorPosition(
        jointAngles: SO101JointAngles,
        baseTransform: simd_double4x4 = matrix_identity_double4x4
    ) -> SIMD3<Double> {
        forwardKinematics(jointAngles: jointAngles, baseTransform: baseTransform).endEffectorPosition
    }

    static func endEffectorPosition(
        jointPositions: [String: Double],
        baseTransform: simd_double4x4 = matrix_identity_double4x4
    ) -> SIMD3<Double> {
        let seededAngles = clampToJointLimits(SO101JointAngles.seeded(from: jointPositions))
        return endEffectorPosition(jointAngles: seededAngles, baseTransform: baseTransform)
    }
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
