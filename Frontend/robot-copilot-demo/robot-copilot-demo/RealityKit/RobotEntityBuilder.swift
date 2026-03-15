import Foundation
import RealityKit
import UIKit

/// Builds the placeholder SO-101 robot entity hierarchy using RealityKit primitives.
/// Swap this for a real USDZ asset when available.
@MainActor
enum RobotEntityBuilder {

    // MARK: - Colors

    private static let bodyColor = SimpleMaterial(color: .init(red: 0.2, green: 0.2, blue: 0.8, alpha: 1), isMetallic: true)
    private static let armColor = SimpleMaterial(color: .init(red: 0.3, green: 0.3, blue: 0.9, alpha: 1), isMetallic: true)
    private static let gripperColor = SimpleMaterial(color: .init(red: 0.8, green: 0.4, blue: 0.1, alpha: 1), isMetallic: false)
    private static let markerColor = SimpleMaterial(color: .init(red: 0.0, green: 1.0, blue: 0.4, alpha: 0.8), isMetallic: false)

    // MARK: - Robot entity

    /// Builds a placeholder SO-101 arm entity hierarchy.
    static func buildRobotEntity() -> Entity {
        let root = Entity()
        root.name = "so101_robot"

        // Base
        let base = ModelEntity(
            mesh: .generateBox(size: [0.08, 0.04, 0.08], cornerRadius: 0.005),
            materials: [bodyColor]
        )
        base.name = "base"
        base.position = [0, 0.02, 0]
        root.addChild(base)

        // Link 1 (shoulder)
        let link1 = ModelEntity(
            mesh: .generateCylinder(height: 0.12, radius: 0.015),
            materials: [armColor]
        )
        link1.name = "link1"
        link1.position = [0, 0.1, 0]
        root.addChild(link1)

        // Link 2 (upper arm)
        let link2 = ModelEntity(
            mesh: .generateCylinder(height: 0.14, radius: 0.012),
            materials: [armColor]
        )
        link2.name = "link2"
        link2.position = [0, 0.22, 0.04]
        link2.orientation = simd_quatf(angle: .pi / 4, axis: [1, 0, 0])
        root.addChild(link2)

        // Link 3 (forearm)
        let link3 = ModelEntity(
            mesh: .generateCylinder(height: 0.12, radius: 0.010),
            materials: [armColor]
        )
        link3.name = "link3"
        link3.position = [0, 0.30, 0.10]
        link3.orientation = simd_quatf(angle: -.pi / 6, axis: [1, 0, 0])
        root.addChild(link3)

        // Gripper body
        let gripperBody = ModelEntity(
            mesh: .generateBox(size: [0.04, 0.02, 0.03], cornerRadius: 0.003),
            materials: [gripperColor]
        )
        gripperBody.name = "gripper"
        gripperBody.position = [0, 0.36, 0.14]
        root.addChild(gripperBody)

        // Gripper fingers
        for xOffset: Float in [-0.015, 0.015] {
            let finger = ModelEntity(
                mesh: .generateBox(size: [0.008, 0.025, 0.008]),
                materials: [gripperColor]
            )
            finger.position = [xOffset, 0.37, 0.155]
            root.addChild(finger)
        }

        // Add interaction components to root
        root.components.set(InputTargetComponent())
        root.generateCollisionShapes(recursive: true)

        return root
    }

    // MARK: - End-effector marker

    /// Glowing sphere at the end-effector position.
    static func buildEndEffectorMarker() -> Entity {
        let marker = ModelEntity(
            mesh: .generateSphere(radius: 0.012),
            materials: [markerColor]
        )
        marker.name = "end_effector_marker"
        marker.position = [0, 0.38, 0.16]
        return marker
    }

    // MARK: - Workspace bounds

    /// Wireframe box showing the reachable workspace.
    static func buildWorkspaceBoundsEntity() -> Entity {
        let bounds = Entity()
        bounds.name = "workspace_bounds"

        let edgeMaterial = SimpleMaterial(color: .init(red: 0.0, green: 0.8, blue: 1.0, alpha: 0.4), isMetallic: false)
        let edgeRadius: Float = 0.003

        // 12 edges of a 0.4 x 0.4 x 0.4 m workspace box
        let w: Float = 0.4
        let h: Float = 0.4
        let d: Float = 0.4
        let cx: Float = 0
        let cy: Float = 0.2
        let cz: Float = 0

        // Horizontal edges (4 bottom, 4 top)
        let horizontalEdges: [(SIMD3<Float>, SIMD3<Float>, Float)] = [
            // Bottom
            ([cx, cy - h/2, cz - d/2], [1, 0, 0], w),
            ([cx, cy - h/2, cz + d/2], [1, 0, 0], w),
            ([cx - w/2, cy - h/2, cz], [0, 0, 1], d),
            ([cx + w/2, cy - h/2, cz], [0, 0, 1], d),
            // Top
            ([cx, cy + h/2, cz - d/2], [1, 0, 0], w),
            ([cx, cy + h/2, cz + d/2], [1, 0, 0], w),
            ([cx - w/2, cy + h/2, cz], [0, 0, 1], d),
            ([cx + w/2, cy + h/2, cz], [0, 0, 1], d),
        ]

        for (pos, axis, length) in horizontalEdges {
            let edge = ModelEntity(mesh: .generateCylinder(height: length, radius: edgeRadius), materials: [edgeMaterial])
            edge.position = pos
            if axis.x != 0 { edge.orientation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1]) }
            bounds.addChild(edge)
        }

        // Vertical edges (4 pillars)
        let verticalPositions: [SIMD3<Float>] = [
            [cx - w/2, cy, cz - d/2],
            [cx + w/2, cy, cz - d/2],
            [cx - w/2, cy, cz + d/2],
            [cx + w/2, cy, cz + d/2],
        ]
        for pos in verticalPositions {
            let edge = ModelEntity(mesh: .generateCylinder(height: h, radius: edgeRadius), materials: [edgeMaterial])
            edge.position = pos
            bounds.addChild(edge)
        }

        return bounds
    }
}
