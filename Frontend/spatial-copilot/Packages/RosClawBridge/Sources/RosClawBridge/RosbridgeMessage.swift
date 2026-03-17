import Foundation

// MARK: - Internal JSON value type (no conflict with OpenClawProtocol.AnyCodable)

/// Internal type-erased Codable value for rosbridge JSON payloads.
/// Uses a typed enum so it is fully `Sendable` without `@unchecked`.
enum _AnyJSON: Codable, Sendable {
    case null
    case bool(Bool)
    case int(Int)
    case double(Double)
    case string(String)
    case object([String: _AnyJSON])
    case array([_AnyJSON])

    init(_ value: Any) {
        switch value {
        case let v as Bool:   self = .bool(v)
        case let v as Int:    self = .int(v)
        case let v as Double: self = .double(v)
        case let v as Float:  self = .double(Double(v))
        case let v as String: self = .string(v)
        case let v as [String: Any]:
            self = .object(v.mapValues { _AnyJSON($0) })
        case let v as [Any]:
            self = .array(v.map { _AnyJSON($0) })
        default:
            self = .null
        }
    }

    /// Extracts the underlying value as `Any` for interop with `[String: Any]` APIs.
    var rawValue: Any {
        switch self {
        case .null:           return NSNull()
        case .bool(let v):    return v
        case .int(let v):     return v
        case .double(let v):  return v
        case .string(let v):  return v
        case .object(let v):  return v.mapValues { $0.rawValue }
        case .array(let v):   return v.map { $0.rawValue }
        }
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.singleValueContainer()
        if c.decodeNil() {
            self = .null
        } else if let v = try? c.decode(Bool.self) {
            self = .bool(v)
        } else if let v = try? c.decode(Int.self) {
            self = .int(v)
        } else if let v = try? c.decode(Double.self) {
            self = .double(v)
        } else if let v = try? c.decode(String.self) {
            self = .string(v)
        } else if let v = try? c.decode([String: _AnyJSON].self) {
            self = .object(v)
        } else if let v = try? c.decode([_AnyJSON].self) {
            self = .array(v)
        } else {
            throw DecodingError.dataCorruptedError(in: c, debugDescription: "Unsupported JSON type")
        }
    }

    func encode(to encoder: Encoder) throws {
        var c = encoder.singleValueContainer()
        switch self {
        case .null:           try c.encodeNil()
        case .bool(let v):    try c.encode(v)
        case .int(let v):     try c.encode(v)
        case .double(let v):  try c.encode(v)
        case .string(let v):  try c.encode(v)
        case .object(let v):  try c.encode(v)
        case .array(let v):   try c.encode(v)
        }
    }
}

// MARK: - Rosbridge protocol message

/// A rosbridge v2 protocol message (JSON envelope).
public struct RosbridgeMessage: Codable, Sendable {
    public var op: String
    public var id: String?
    public var topic: String?
    public var type: String?
    var msg: _AnyJSON?
    public var service: String?
    var args: _AnyJSON?
    public var action: String?
    public var action_type: String?
    public var result: Bool?
    var values: _AnyJSON?
    var feedback: _AnyJSON?

    public init(op: String, id: String? = nil) {
        self.op = op
        self.id = id
    }
}

// MARK: - Connection status

public enum ConnectionStatus: String, Sendable {
    case disconnected
    case connecting
    case connected
}

// MARK: - Topic info

public struct TopicInfo: Sendable {
    public let name: String
    public let type: String
    public init(name: String, type: String) {
        self.name = name
        self.type = type
    }
}
