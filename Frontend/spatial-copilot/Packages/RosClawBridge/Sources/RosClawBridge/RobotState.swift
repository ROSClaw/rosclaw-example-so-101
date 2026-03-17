import Foundation

// MARK: - Robot state model (§11.2)

public enum ConnectionState: String, Sendable {
    case disconnected
    case connecting
    case connected
}

public enum GripperState: String, Sendable {
    case unknown
    case open
    case closed
    case moving
}

public enum RobotMode: String, Sendable {
    case unknown
    case idle
    case executing
    case homing
    case teaching
    case faulted
}

public enum ActionStatus: String, Sendable {
    case idle
    case executing
    case succeeded
    case failed
    case cancelled
}

public struct WorkspaceBounds: Sendable, Codable, Equatable {
    public var xMin: Double
    public var xMax: Double
    public var yMin: Double
    public var yMax: Double
    public var zMin: Double
    public var zMax: Double

    public init(
        xMin: Double,
        xMax: Double,
        yMin: Double,
        yMax: Double,
        zMin: Double,
        zMax: Double)
    {
        self.xMin = xMin
        self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax
        self.zMin = zMin
        self.zMax = zMax
    }

    public static let followerDefault = WorkspaceBounds(
        xMin: -0.3,
        xMax: 0.5,
        yMin: -0.4,
        yMax: 0.4,
        zMin: 0.0,
        zMax: 0.5
    )
}

public struct RobotVector3: Sendable, Codable, Equatable {
    public var x: Double
    public var y: Double
    public var z: Double

    public init(x: Double, y: Double, z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }
}

public struct RobotQuaternion: Sendable, Codable, Equatable {
    public var x: Double
    public var y: Double
    public var z: Double
    public var w: Double

    public init(x: Double, y: Double, z: Double, w: Double) {
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    }
}

public struct RobotToolPose: Sendable, Codable, Equatable {
    public var frameID: String?
    public var position: RobotVector3?
    public var orientation: RobotQuaternion?

    public init(
        frameID: String? = nil,
        position: RobotVector3? = nil,
        orientation: RobotQuaternion? = nil)
    {
        self.frameID = frameID
        self.position = position
        self.orientation = orientation
    }

    enum CodingKeys: String, CodingKey {
        case frameID = "frame_id"
        case position
        case orientation
    }
}

/// Live robot state maintained by the app (§11.2).
public struct RobotState: Sendable {
    public var connectionState: ConnectionState = .disconnected
    public var enabled: Bool = false
    public var jointOrder: [String] = []
    public var jointPositions: [String: Double] = [:]
    public var gripperPosition: Double? = nil
    public var gripperOpenFraction: Double? = nil
    public var toolPose: RobotToolPose? = nil
    public var gripperState: GripperState = .unknown
    public var mode: RobotMode = .unknown
    public var fault: String? = nil
    public var actionStatus: ActionStatus = .idle
    public var estopLatched: Bool = false
    public var lastCameraFrameTimestamp: Date? = nil

    public init() {}

    public var jointSummary: [String: Double] {
        get {
            if !jointPositions.isEmpty {
                return jointPositions
            }
            return [:]
        }
        set {
            self.jointPositions = newValue
            if self.jointOrder.isEmpty {
                self.jointOrder = newValue.keys.sorted()
            }
        }
    }

    /// True when the robot is safe to receive motion commands.
    public var canExecuteMotion: Bool {
        connectionState == .connected && enabled && !estopLatched && fault == nil && mode != .faulted
    }
}

// MARK: - Safety result

public enum SafetyResult: Sendable, Equatable {
    case allowed
    case blocked(reason: String)

    public var isAllowed: Bool {
        if case .allowed = self { return true }
        return false
    }

    public var blockReason: String? {
        if case .blocked(let reason) = self { return reason }
        return nil
    }
}

// MARK: - Command preview model (§11.3)

public struct CommandPreview: Sendable {
    public var parsedCommandName: String
    public var targets: [String]
    public var validationResult: SafetyResult
    public var subsystemsTouched: [String]
    public var requiresConfirmation: Bool
    public var canExecute: Bool

    public init(
        parsedCommandName: String,
        targets: [String],
        validationResult: SafetyResult,
        subsystemsTouched: [String],
        requiresConfirmation: Bool,
        canExecute: Bool
    ) {
        self.parsedCommandName = parsedCommandName
        self.targets = targets
        self.validationResult = validationResult
        self.subsystemsTouched = subsystemsTouched
        self.requiresConfirmation = requiresConfirmation
        self.canExecute = canExecute
    }
}
