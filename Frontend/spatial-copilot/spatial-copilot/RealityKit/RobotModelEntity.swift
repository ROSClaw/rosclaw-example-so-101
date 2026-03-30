import Foundation
import RealityKit
import RosClawBridge
internal import UIKit

/// Loads the packaged SO-101 USD model and provides live + preview-driven joint updates.
@MainActor
final class RobotModelEntity {

    let rootEntity: Entity
    private let visualRoot: Entity
    private(set) var didLoadPackagedAsset = false
    private var liveJointPositions: [String: Double] = [:]
    private var previewJointPositions: [String: Double]?
    private var bodyEntities: [String: Entity] = [:]
    private var currentGripperAngle: Double = SO101Kinematics.jawAngle(forOpenFraction: nil)
    private var currentForwardKinematics: SO101ForwardKinematics?

    init() async {
        rootEntity = Entity()
        if let loaded = await Self.loadPackagedRobot() {
            loaded.transform = Transform(matrix: SO101VisualAlignment.kinematicBaseFromModelTransform)
            visualRoot = loaded
            didLoadPackagedAsset = true
        } else {
            visualRoot = Self.buildPrimitiveFallback()
        }
        rootEntity.addChild(visualRoot)
        discoverBodies(in: visualRoot)
        rootEntity.name = "so101_robot"
        rootEntity.components.set(InputTargetComponent())
        rootEntity.generateCollisionShapes(recursive: true)
        applyCurrentPose()
    }

    // MARK: - Joint updates

    func updateJoints(from robotState: RobotState) {
        liveJointPositions = robotState.jointPositions
        guard previewJointPositions == nil else { return }
        applyCurrentPose()
    }

    func applyPreviewJoints(_ jointPositions: [String: Double]?) {
        previewJointPositions = jointPositions
        applyCurrentPose()
    }

    func updateGripper(openFraction: Double?) {
        currentGripperAngle = SO101Kinematics.jawAngle(forOpenFraction: openFraction)
        applyCurrentPose()
    }

    func setFaulted(_ faulted: Bool) {
        rootEntity.components.set(OpacityComponent(opacity: faulted ? 0.82 : 1.0))
    }

    var endEffectorPosition: SIMD3<Float> {
        guard let currentForwardKinematics else { return [0.20, 0.00, 0.18] }
        let position = currentForwardKinematics.endEffectorPosition
        return SIMD3<Float>(Float(position.x), Float(position.y), Float(position.z))
    }

    var hasExpectedBodyBindings: Bool {
        SO101Kinematics.bodyHierarchy.allSatisfy { bodyEntities[$0] != nil }
    }

    func setCalibrationTransform(_ transform: simd_float4x4) {
        rootEntity.transform = Transform(matrix: transform)
    }

    private func applyCurrentPose() {
        let effectiveJointPositions = previewJointPositions ?? liveJointPositions
        let jointAngles = SO101Kinematics.clampToJointLimits(
            SO101JointAngles.seeded(from: effectiveJointPositions)
        )
        let forwardKinematics = SO101Kinematics.forwardKinematics(
            jointAngles: jointAngles,
            gripperAngle: currentGripperAngle
        )
        currentForwardKinematics = forwardKinematics

        for (bodyName, entity) in bodyEntities {
            guard let transform = forwardKinematics.bodyModelTransforms[bodyName] else { continue }
            entity.transform = Transform(matrix: simd_float4x4(transform))
        }
    }

    private func discoverBodies(in entity: Entity) {
        var bestPaths: [String: Int] = [:]

        func walk(_ current: Entity, path: [String]) {
            let normalizedName = current.name.replacingOccurrences(of: "-", with: "_").lowercased()
            let normalizedPath = path.map { $0.replacingOccurrences(of: "-", with: "_").lowercased() }
            if SO101Kinematics.bodyHierarchy.contains(normalizedName),
               !normalizedPath.contains("visuals"),
               !normalizedPath.contains("collisions") {
                let depth = normalizedPath.count
                if bestPaths[normalizedName] == nil || depth < bestPaths[normalizedName, default: depth + 1] {
                    bestPaths[normalizedName] = depth
                    bodyEntities[normalizedName] = current
                }
            }

            for child in current.children {
                walk(child, path: path + [current.name])
            }
        }

        walk(entity, path: [])
    }

    private static func loadPackagedRobot() async -> Entity? {
        for url in resourceURLs() {
            if let loaded = try? await Entity(contentsOf: url) {
                return loaded
            }
        }
        return nil
    }

    private static func resourceURLs() -> [URL] {
        let bundles = [Bundle.main] + Bundle.allBundles + Bundle.allFrameworks
        var urls: [URL] = []

        for bundle in bundles {
            urls.append(contentsOf: [
                bundle.url(forResource: "so101_follower", withExtension: "usd"),
                bundle.url(forResource: "so101_follower", withExtension: "usd", subdirectory: "Assets"),
                bundle.url(forResource: "so101_follower", withExtension: "usd", subdirectory: "Resources"),
            ].compactMap { $0 })
        }

        return Array(Set(urls))
    }

    private static func buildPrimitiveFallback() -> Entity {
        let root = Entity()
        let body = SimpleMaterial(color: .init(red: 0.2, green: 0.2, blue: 0.8, alpha: 1), isMetallic: true)
        let arm = SimpleMaterial(color: .init(red: 0.3, green: 0.3, blue: 0.9, alpha: 1), isMetallic: true)
        let grip = SimpleMaterial(color: .init(red: 0.8, green: 0.4, blue: 0.1, alpha: 1), isMetallic: false)
        let base = ModelEntity(mesh: .generateBox(size: [0.08, 0.04, 0.08], cornerRadius: 0.005), materials: [body])
        base.name = "base"; base.position = [0, 0.02, 0]; root.addChild(base)
        let l1 = ModelEntity(mesh: .generateCylinder(height: 0.12, radius: 0.015), materials: [arm])
        l1.name = "shoulder"; l1.position = [0, 0.1, 0]; root.addChild(l1)
        let l2 = ModelEntity(mesh: .generateCylinder(height: 0.14, radius: 0.012), materials: [arm])
        l2.name = "upper_arm"; l2.position = [0.07, 0.22, 0.04]; l2.orientation = simd_quatf(angle: .pi / 4, axis: [0, 0, 1]); root.addChild(l2)
        let l3 = ModelEntity(mesh: .generateCylinder(height: 0.12, radius: 0.010), materials: [arm])
        l3.name = "lower_arm"; l3.position = [0.14, 0.30, 0.10]; l3.orientation = simd_quatf(angle: -.pi / 6, axis: [0, 0, 1]); root.addChild(l3)
        let wrist = ModelEntity(mesh: .generateCylinder(height: 0.08, radius: 0.009), materials: [arm])
        wrist.name = "wrist"; wrist.position = [0.19, 0.34, 0.14]; wrist.orientation = simd_quatf(angle: -.pi / 3, axis: [0, 0, 1]); root.addChild(wrist)
        let gb = ModelEntity(mesh: .generateBox(size: [0.04, 0.02, 0.03], cornerRadius: 0.003), materials: [grip])
        gb.name = "gripper"; gb.position = [0.24, 0.36, 0.16]; root.addChild(gb)
        let jaw = ModelEntity(mesh: .generateBox(size: [0.008, 0.025, 0.008]), materials: [grip])
        jaw.name = "jaw"; jaw.position = [0.27, 0.37, 0.165]; root.addChild(jaw)
        return root
    }
}

private extension simd_float4x4 {
    init(_ matrix: simd_double4x4) {
        self.init(
            SIMD4<Float>(Float(matrix.columns.0.x), Float(matrix.columns.0.y), Float(matrix.columns.0.z), Float(matrix.columns.0.w)),
            SIMD4<Float>(Float(matrix.columns.1.x), Float(matrix.columns.1.y), Float(matrix.columns.1.z), Float(matrix.columns.1.w)),
            SIMD4<Float>(Float(matrix.columns.2.x), Float(matrix.columns.2.y), Float(matrix.columns.2.z), Float(matrix.columns.2.w)),
            SIMD4<Float>(Float(matrix.columns.3.x), Float(matrix.columns.3.y), Float(matrix.columns.3.z), Float(matrix.columns.3.w))
        )
    }
}
