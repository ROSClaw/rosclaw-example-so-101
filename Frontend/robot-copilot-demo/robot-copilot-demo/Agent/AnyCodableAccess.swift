import Foundation
import OpenClawProtocol

extension AnyCodable {
    nonisolated var objectValue: [String: AnyCodable]? {
        guard case .object(let object) = value else {
            return nil
        }
        return object
    }

    nonisolated var arrayValue: [AnyCodable]? {
        guard case .array(let array) = value else {
            return nil
        }
        return array
    }

    nonisolated var stringValue: String? {
        guard case .string(let string) = value else {
            return nil
        }
        return string
    }

    nonisolated var boolValue: Bool? {
        guard case .bool(let bool) = value else {
            return nil
        }
        return bool
    }

    nonisolated var doubleValue: Double? {
        switch value {
        case .double(let number):
            return number
        case .int(let number):
            return Double(number)
        default:
            return nil
        }
    }
}
