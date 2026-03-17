//
//  SIMD3+Extension.swift
//  spatial-copilot
//
//  Created by Marcus Arnett on 3/13/26.
//

import SwiftUI

public extension SIMD3<Float> {
    init(_ point: Point3D) {
        self.init(
            x: .init(point.x),
            y: .init(point.y),
            z: .init(point.z)
        )
    }
}
