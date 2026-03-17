import Foundation

public enum RobotCartesianUnit: String, Sendable, Codable, Equatable {
    case millimeters = "mm"
    case centimeters = "cm"
    case meters = "m"

    public var symbol: String { rawValue }

    public func toMeters(_ value: Double) -> Double {
        switch self {
        case .millimeters:
            return value / 1000.0
        case .centimeters:
            return value / 100.0
        case .meters:
            return value
        }
    }

    public func toCentimeters(_ value: Double) -> Double {
        switch self {
        case .millimeters:
            return value / 10.0
        case .centimeters:
            return value
        case .meters:
            return value * 100.0
        }
    }
}

public enum RobotCartesianReferenceFrame: String, Sendable, Codable, Equatable {
    case baseAbsolute
    case toolRelative

    public var displayName: String {
        switch self {
        case .baseAbsolute:
            return "Base absolute"
        case .toolRelative:
            return "Tool-relative"
        }
    }
}

public enum RobotCartesianSource: String, Sendable, Codable, Equatable {
    case voice
    case spatial
}

public struct RobotCartesianTarget: Sendable, Codable, Equatable {
    public var x: Double
    public var y: Double
    public var z: Double
    public var unit: RobotCartesianUnit
    public var referenceFrame: RobotCartesianReferenceFrame
    public var source: RobotCartesianSource

    public init(
        x: Double,
        y: Double,
        z: Double,
        unit: RobotCartesianUnit,
        referenceFrame: RobotCartesianReferenceFrame,
        source: RobotCartesianSource
    ) {
        self.x = x
        self.y = y
        self.z = z
        self.unit = unit
        self.referenceFrame = referenceFrame
        self.source = source
    }

    public var requestedVector: RobotVector3 {
        RobotVector3(x: x, y: y, z: z)
    }

    public var requestedMeters: RobotVector3 {
        RobotVector3(
            x: unit.toMeters(x),
            y: unit.toMeters(y),
            z: unit.toMeters(z)
        )
    }

    public var requestedCentimeters: RobotVector3 {
        RobotVector3(
            x: unit.toCentimeters(x),
            y: unit.toCentimeters(y),
            z: unit.toCentimeters(z)
        )
    }

    public var frameDisplayName: String {
        referenceFrame.displayName
    }

    public var requestSummary: String {
        String(
            format: "(%.3f %@, %.3f %@, %.3f %@)",
            x,
            unit.symbol,
            y,
            unit.symbol,
            z,
            unit.symbol
        )
    }

    public var centimeterSummary: String {
        let centimeters = requestedCentimeters
        return String(
            format: "(%.3f cm, %.3f cm, %.3f cm)",
            centimeters.x,
            centimeters.y,
            centimeters.z
        )
    }
}

public struct RobotCartesianAxisRange: Sendable, Codable, Equatable {
    public var minCm: Double
    public var maxCm: Double

    public init(minCm: Double, maxCm: Double) {
        self.minCm = minCm
        self.maxCm = maxCm
    }
}

public struct RobotCartesianSafeRange: Sendable, Codable, Equatable {
    public var referenceFrame: RobotCartesianReferenceFrame
    public var x: RobotCartesianAxisRange
    public var y: RobotCartesianAxisRange
    public var z: RobotCartesianAxisRange

    public init(
        referenceFrame: RobotCartesianReferenceFrame,
        x: RobotCartesianAxisRange,
        y: RobotCartesianAxisRange,
        z: RobotCartesianAxisRange
    ) {
        self.referenceFrame = referenceFrame
        self.x = x
        self.y = y
        self.z = z
    }
}
