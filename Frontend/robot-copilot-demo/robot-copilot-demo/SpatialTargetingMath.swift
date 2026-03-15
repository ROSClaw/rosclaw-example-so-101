import simd

enum SpatialTargetingMath {
    static func robotPosition(
        fromWorldPosition worldPosition: SIMD3<Float>,
        relativeToRobotBase baseWorldPosition: SIMD3<Float>
    ) -> SIMD3<Float> {
        let relative = worldPosition - baseWorldPosition

        // visionOS: +X right, +Y up, +Z toward the user
        // ROS base_link: +X forward, +Y left, +Z up
        return SIMD3<Float>(
            -relative.z,
            -relative.x,
            relative.y
        )
    }
}
