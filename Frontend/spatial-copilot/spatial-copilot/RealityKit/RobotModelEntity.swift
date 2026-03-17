import Foundation
import RealityKit
import RosClawBridge
internal import UIKit

/// Loads the packaged SO-101 USD model and provides live + preview-driven joint updates.
@MainActor
final class RobotModelEntity {

    let rootEntity: Entity
    private(set) var didLoadPackagedAsset = false
    private var liveJointPositions: [String: Double] = [:]
    private var previewJointPositions: [String: Double]?
    private var bodyEntities: [String: Entity] = [:]
    private var restTransforms: [String: Transform] = [:]
    private var jawEntity: Entity?
    private var currentCalibrationTransform = matrix_identity_float4x4

    private let jointToBodyName: [String: String] = [
        SO101ArmJoint.shoulderPan.rawValue: "shoulder",
        SO101ArmJoint.shoulderLift.rawValue: "upper_arm",
        SO101ArmJoint.elbowFlex.rawValue: "lower_arm",
        SO101ArmJoint.wristFlex.rawValue: "wrist",
        SO101ArmJoint.wristRoll.rawValue: "gripper",
    ]

    init() async {
        if let loaded = await Self.loadPackagedRobot() {
            rootEntity = loaded
            didLoadPackagedAsset = true
            discoverBodies(in: loaded)
        } else {
            rootEntity = Self.buildPrimitiveFallback()
            discoverBodies(in: rootEntity)
        }
        rootEntity.name = "so101_robot"
        rootEntity.components.set(InputTargetComponent())
        rootEntity.generateCollisionShapes(recursive: true)
    }

    // MARK: - Joint updates

    func updateJoints(from robotState: RobotState) {
        liveJointPositions = robotState.jointPositions
        guard previewJointPositions == nil else { return }
        applyJointMap(robotState.jointPositions)
    }

    func applyPreviewJoints(_ jointPositions: [String: Double]?) {
        previewJointPositions = jointPositions
        if let jointPositions {
            applyJointMap(jointPositions)
        } else {
            applyJointMap(liveJointPositions)
        }
    }

    func updateGripper(openFraction: Double?) {
        guard let jawEntity, let rest = restTransforms["jaw"] else { return }
        let fraction = Float(openFraction ?? 0.5)
        let angle = (-10.0 + (110.0 * fraction)) * (.pi / 180.0)
        jawEntity.transform = Transform(
            scale: rest.scale,
            rotation: simd_quatf(angle: angle, axis: SIMD3<Float>(0, 0, 1)) * rest.rotation,
            translation: rest.translation
        )
    }

    func setFaulted(_ faulted: Bool) {
        rootEntity.components.set(OpacityComponent(opacity: faulted ? 0.82 : 1.0))
    }

    var endEffectorPosition: SIMD3<Float> {
        jawEntity?.position(relativeTo: rootEntity) ?? [0.20, 0.00, 0.18]
    }

    var hasExpectedBodyBindings: Bool {
        SO101Kinematics.bodyHierarchy.allSatisfy { bodyEntities[$0] != nil }
    }

    func setCalibrationTransform(_ transform: simd_float4x4) {
        currentCalibrationTransform = transform
        rootEntity.transform = Transform(matrix: transform)
    }

    private func applyJointMap(_ jointPositions: [String: Double]) {
        for joint in SO101ArmJoint.allCases {
            guard let bodyName = jointToBodyName[joint.rawValue],
                  let entity = bodyEntities[bodyName],
                  let rest = restTransforms[bodyName] else {
                continue
            }

            let axis = axisForJoint(joint)
            let angle = Float(jointPositions[joint.rawValue] ?? 0)
            entity.transform = Transform(
                scale: rest.scale,
                rotation: simd_quatf(angle: angle, axis: axis) * rest.rotation,
                translation: rest.translation
            )
        }
    }

    private func axisForJoint(_ joint: SO101ArmJoint) -> SIMD3<Float> {
        let axis = SO101Kinematics.metadata(for: joint).axisInJointFrame
        return SIMD3<Float>(Float(axis.x), Float(axis.y), Float(axis.z))
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
                    restTransforms[normalizedName] = current.transform
                    if normalizedName == "jaw" {
                        jawEntity = current
                    }
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
