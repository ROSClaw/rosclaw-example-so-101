import ARKit
import Foundation
import RealityKit
import simd

struct SpatialPlaneMesh {
    struct Triangle {
        let a: SIMD2<Float>
        let b: SIMD2<Float>
        let c: SIMD2<Float>

        var area: Float {
            abs(crossZ(b - a, c - a)) * 0.5
        }
    }

    let anchorID: UUID
    let worldFromPlaneTransform: simd_float4x4
    let surfaceNormal: SIMD3<Float>
    let classificationBias: Float
    let triangles: [Triangle]
    let areaEstimate: Float

    init(anchor: PlaneAnchor) {
        var triangles: [Triangle] = []
        triangles.reserveCapacity(anchor.geometry.meshFaces.count)

        for faceIndex in 0..<anchor.geometry.meshFaces.count {
            let vertexIndices = anchor.geometry.meshFaces[faceIndex]
            guard vertexIndices.count == 3 else { continue }

            let vertex1 = anchor.geometry.meshVertices[vertexIndices[0]]
            let vertex2 = anchor.geometry.meshVertices[vertexIndices[1]]
            let vertex3 = anchor.geometry.meshVertices[vertexIndices[2]]

            let triangle = Triangle(
                a: SIMD2<Float>(vertex1.0, vertex1.2),
                b: SIMD2<Float>(vertex2.0, vertex2.2),
                c: SIMD2<Float>(vertex3.0, vertex3.2)
            )
            guard triangle.area > 0.000001 else { continue }
            triangles.append(triangle)
        }

        var normal = SIMD3<Float>(
            anchor.originFromAnchorTransform.columns.1.x,
            anchor.originFromAnchorTransform.columns.1.y,
            anchor.originFromAnchorTransform.columns.1.z
        )
        if simd_dot(normal, SIMD3<Float>(0, 1, 0)) < 0 {
            normal *= -1
        }

        self.init(
            anchorID: anchor.id,
            worldFromPlaneTransform: anchor.originFromAnchorTransform,
            surfaceNormal: simd_normalize(normal),
            classificationBias: anchor.surfaceClassification == .table ? 0.0 : 0.12,
            triangles: triangles
        )
    }

    init(
        anchorID: UUID = UUID(),
        worldFromPlaneTransform: simd_float4x4,
        surfaceNormal: SIMD3<Float>,
        classificationBias: Float = 0.0,
        triangles: [Triangle]
    ) {
        self.anchorID = anchorID
        self.worldFromPlaneTransform = worldFromPlaneTransform
        self.surfaceNormal = simd_normalize(surfaceNormal)
        self.classificationBias = classificationBias
        self.triangles = triangles
        self.areaEstimate = max(triangles.reduce(0) { $0 + $1.area }, 0.0001)
    }
}

enum SpatialPlaneProjector {
    static let defaultContainmentEpsilon: Float = 0.02
    static let defaultAgreementThreshold: Float = 0.05

    struct ProjectionCandidate {
        let anchorID: UUID
        let worldPoint: SIMD3<Float>
        let surfaceNormal: SIMD3<Float>
        let distanceToPoint: Float
        let surfaceArea: Float
        let classificationBias: Float
    }

    struct RaycastCandidate {
        let anchorID: UUID
        let worldPoint: SIMD3<Float>
        let surfaceNormal: SIMD3<Float>
        let distanceToPoint: Float
        let surfaceArea: Float
        let classificationBias: Float
    }

    enum ResolutionMethod {
        case raycast
        case meshProjection
    }

    struct ResolvedCandidate {
        let anchorID: UUID
        let worldPoint: SIMD3<Float>
        let surfaceNormal: SIMD3<Float>
        let distanceToPoint: Float
        let score: Float
        let method: ResolutionMethod
        let comparisonDeltaMeters: Float?
    }

    static func project(
        point: SIMD3<Float>,
        onto planeMeshes: [SpatialPlaneMesh],
        maxDistance: Float,
        containmentEpsilon: Float = defaultContainmentEpsilon
    ) -> [ProjectionCandidate] {
        planeMeshes.compactMap { mesh in
            project(
                point: point,
                onto: mesh,
                maxDistance: maxDistance,
                containmentEpsilon: containmentEpsilon
            )
        }
    }

    static func project(
        point: SIMD3<Float>,
        onto planeMesh: SpatialPlaneMesh,
        maxDistance: Float,
        containmentEpsilon: Float = defaultContainmentEpsilon
    ) -> ProjectionCandidate? {
        let planeFromWorldTransform = planeMesh.worldFromPlaneTransform.inverse
        let localPoint = SpatialTargetingMath.transform(point: point, with: planeFromWorldTransform)
        let orthogonalDistance = abs(localPoint.y)
        guard orthogonalDistance <= maxDistance else {
            return nil
        }

        let projectedPoint = SIMD2<Float>(localPoint.x, localPoint.z)
        guard contains(projectedPoint: projectedPoint, in: planeMesh.triangles, epsilon: containmentEpsilon) else {
            return nil
        }

        let projectedLocalPoint = SIMD3<Float>(projectedPoint.x, 0, projectedPoint.y)
        let worldPoint = SpatialTargetingMath.transform(point: projectedLocalPoint, with: planeMesh.worldFromPlaneTransform)

        return ProjectionCandidate(
            anchorID: planeMesh.anchorID,
            worldPoint: worldPoint,
            surfaceNormal: planeMesh.surfaceNormal,
            distanceToPoint: simd_length(worldPoint - point),
            surfaceArea: planeMesh.areaEstimate,
            classificationBias: planeMesh.classificationBias
        )
    }

    static func contains(
        projectedPoint: SIMD2<Float>,
        in triangles: [SpatialPlaneMesh.Triangle],
        epsilon: Float = defaultContainmentEpsilon
    ) -> Bool {
        triangles.contains { triangle in
            projectedPoint.isInsideTriangle(triangle, epsilon: epsilon)
        }
    }

    static func resolvePreferredCandidate(
        raycastCandidate: RaycastCandidate?,
        projectionCandidates: [ProjectionCandidate],
        agreementThreshold: Float = defaultAgreementThreshold
    ) -> ResolvedCandidate? {
        let bestProjection = projectionCandidates
            .map { candidate in
                ResolvedCandidate(
                    anchorID: candidate.anchorID,
                    worldPoint: candidate.worldPoint,
                    surfaceNormal: candidate.surfaceNormal,
                    distanceToPoint: candidate.distanceToPoint,
                    score: score(
                        distanceToPoint: candidate.distanceToPoint,
                        surfaceArea: candidate.surfaceArea,
                        classificationBias: candidate.classificationBias
                    ),
                    method: .meshProjection,
                    comparisonDeltaMeters: nil
                )
            }
            .min(by: { $0.score < $1.score })

        let resolvedRaycast = raycastCandidate.map { candidate in
            ResolvedCandidate(
                anchorID: candidate.anchorID,
                worldPoint: candidate.worldPoint,
                surfaceNormal: candidate.surfaceNormal,
                distanceToPoint: candidate.distanceToPoint,
                score: score(
                    distanceToPoint: candidate.distanceToPoint,
                    surfaceArea: candidate.surfaceArea,
                    classificationBias: candidate.classificationBias
                ),
                method: .raycast,
                comparisonDeltaMeters: nil
            )
        }

        switch (resolvedRaycast, bestProjection) {
        case let (raycast?, projection?):
            let delta = simd_length(raycast.worldPoint - projection.worldPoint)
            let agrees = delta <= agreementThreshold || raycast.anchorID == projection.anchorID
            let chosen: ResolvedCandidate
            if agrees || raycast.score <= projection.score {
                chosen = raycast
            } else {
                chosen = projection
            }
            return ResolvedCandidate(
                anchorID: chosen.anchorID,
                worldPoint: chosen.worldPoint,
                surfaceNormal: chosen.surfaceNormal,
                distanceToPoint: chosen.distanceToPoint,
                score: chosen.score,
                method: chosen.method,
                comparisonDeltaMeters: delta
            )
        case let (raycast?, nil):
            return raycast
        case let (nil, projection?):
            return projection
        case (nil, nil):
            return nil
        }
    }

    private static func score(
        distanceToPoint: Float,
        surfaceArea: Float,
        classificationBias: Float
    ) -> Float {
        distanceToPoint + classificationBias - min(surfaceArea, 1.2) * 0.06
    }
}

extension GeometrySource {
    func asSIMD3FloatArray() -> [SIMD3<Float>] {
        (0..<count).map { index in
            let value = self[Int32(index)]
            return SIMD3<Float>(value.0, value.1, value.2)
        }
    }

    subscript(_ index: Int32) -> (Float, Float, Float) {
        precondition(format == .float3, "Expected plane mesh vertices to use float3 data.")
        return buffer.contents()
            .advanced(by: offset + (stride * Int(index)))
            .assumingMemoryBound(to: (Float, Float, Float).self)
            .pointee
    }
}

extension GeometryElement {
    func asUInt16Array() -> [UInt16] {
        var data: [UInt16] = []
        data.reserveCapacity(count * primitive.indexCount)
        for faceIndex in 0..<count {
            for index in self[faceIndex] {
                data.append(UInt16(index))
            }
        }
        return data
    }

    subscript(_ index: Int) -> [Int32] {
        precondition(bytesPerIndex == MemoryLayout<Int32>.size, "Expected plane mesh faces to use Int32 indices.")
        var indices: [Int32] = []
        indices.reserveCapacity(primitive.indexCount)
        for offset in 0..<primitive.indexCount {
            let pointer = buffer.contents()
                .advanced(by: (index * primitive.indexCount + offset) * MemoryLayout<Int32>.size)
                .assumingMemoryBound(to: Int32.self)
            indices.append(pointer.pointee)
        }
        return indices
    }
}

private extension SIMD2 where Scalar == Float {
    func isInsideTriangle(_ triangle: SpatialPlaneMesh.Triangle, epsilon: Float) -> Bool {
        let barycentric = barycentricCoordinates(in: triangle)
        guard barycentric.x.isFinite, barycentric.y.isFinite, barycentric.z.isFinite else {
            return false
        }
        return barycentric.x >= -epsilon &&
            barycentric.y >= -epsilon &&
            barycentric.z >= -epsilon &&
            barycentric.x <= 1 + epsilon &&
            barycentric.y <= 1 + epsilon &&
            barycentric.z <= 1 + epsilon
    }

    func barycentricCoordinates(in triangle: SpatialPlaneMesh.Triangle) -> SIMD3<Float> {
        let v2FromV1 = triangle.b - triangle.a
        let v3FromV1 = triangle.c - triangle.a
        let selfFromV1 = self - triangle.a
        let overallArea = crossZ(v2FromV1, v3FromV1)
        guard abs(overallArea) > 0.000001 else {
            return SIMD3<Float>(repeating: .infinity)
        }

        let areaU = crossZ(selfFromV1, v3FromV1)
        let areaV = crossZ(v2FromV1, selfFromV1)
        let u = areaU / overallArea
        let v = areaV / overallArea
        let w = 1.0 - v - u
        return SIMD3<Float>(u, v, w)
    }
}

private func crossZ(_ lhs: SIMD2<Float>, _ rhs: SIMD2<Float>) -> Float {
    (lhs.x * rhs.y) - (lhs.y * rhs.x)
}

extension PlaneAnchor {
    static let horizontalCollisionGroup = CollisionGroup(rawValue: 1 << 31)
    static let verticalCollisionGroup = CollisionGroup(rawValue: 1 << 30)
    static let allPlanesCollisionGroup = CollisionGroup(
        rawValue: horizontalCollisionGroup.rawValue | verticalCollisionGroup.rawValue
    )
}
