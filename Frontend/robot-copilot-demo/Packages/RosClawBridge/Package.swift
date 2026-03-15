// swift-tools-version: 6.2

import PackageDescription

let package = Package(
    name: "RosClawBridge",
    platforms: [
        .visionOS(.v26),
        .iOS(.v17),
        .macOS(.v14),
    ],
    products: [
        .library(name: "RosClawBridge", targets: ["RosClawBridge"]),
    ],
    targets: [
        .target(
            name: "RosClawBridge",
            swiftSettings: [
                .enableUpcomingFeature("StrictConcurrency"),
            ]
        ),
    ]
)
