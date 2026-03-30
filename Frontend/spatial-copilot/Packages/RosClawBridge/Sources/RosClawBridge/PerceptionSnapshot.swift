import Foundation

/// A single detected object from the perception pipeline.
public struct PerceptionObject: Sendable, Equatable {
    public let label: String
    public let position: String
    public let distanceM: Double

    public init(label: String, position: String = "", distanceM: Double = 0.0) {
        self.label = label
        self.position = position
        self.distanceM = distanceM
    }
}

/// A detected obstacle from the perception pipeline.
public struct PerceptionObstacle: Sendable, Equatable {
    public let label: String
    public let direction: String
    public let distanceM: Double

    public init(label: String, direction: String = "", distanceM: Double = 0.0) {
        self.label = label
        self.direction = direction
        self.distanceM = distanceM
    }
}

/// Snapshot from the `/rosclaw/perception` topic.
public struct PerceptionSnapshot: Sendable, Equatable {
    public let stamp: Date
    public let summary: String
    public let imageMimeType: String
    public let imageData: Data
    public let objects: [PerceptionObject]
    public let obstacles: [PerceptionObstacle]
    public let alerts: [String]

    public init(
        stamp: Date = Date(),
        summary: String = "",
        imageMimeType: String = "image/jpeg",
        imageData: Data = Data(),
        objects: [PerceptionObject] = [],
        obstacles: [PerceptionObstacle] = [],
        alerts: [String] = []
    ) {
        self.stamp = stamp
        self.summary = summary
        self.imageMimeType = imageMimeType
        self.imageData = imageData
        self.objects = objects
        self.obstacles = obstacles
        self.alerts = alerts
    }
}
