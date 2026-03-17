import Foundation
import RosClawBridge

enum CartesianVoiceCommandSupport {
    static func decodeMetricUnit(raw: String?) -> RobotCartesianUnit? {
        switch raw?.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() {
        case "mm", "millimeter", "millimeters", "millimetre", "millimetres":
            return .millimeters
        case "cm", "centimeter", "centimeters", "centimetre", "centimetres":
            return .centimeters
        case "m", "meter", "meters", "metre", "metres":
            return .meters
        default:
            return nil
        }
    }

    static func detectMetricUnit(in text: String) -> RobotCartesianUnit? {
        let normalized = text
            .lowercased()
            .replacingOccurrences(of: "[^a-z0-9]+", with: " ", options: .regularExpression)
        let tokens = normalized.split(separator: " ").map(String.init)
        let tokenSet = Set(tokens)

        if tokenSet.contains("mm")
            || tokenSet.contains("millimeter")
            || tokenSet.contains("millimeters")
            || tokenSet.contains("millimetre")
            || tokenSet.contains("millimetres")
        {
            return .millimeters
        }
        if tokenSet.contains("cm")
            || tokenSet.contains("centimeter")
            || tokenSet.contains("centimeters")
            || tokenSet.contains("centimetre")
            || tokenSet.contains("centimetres")
        {
            return .centimeters
        }
        if tokenSet.contains("m")
            || tokenSet.contains("meter")
            || tokenSet.contains("meters")
            || tokenSet.contains("metre")
            || tokenSet.contains("metres")
        {
            return .meters
        }
        return nil
    }

    static func finalizeDraft(
        _ draft: PendingCartesianClarificationDraft,
        with answer: String
    ) -> RobotCartesianTarget? {
        guard let unit = detectMetricUnit(in: answer) else {
            return nil
        }
        return RobotCartesianTarget(
            x: draft.x,
            y: draft.y,
            z: draft.z,
            unit: unit,
            referenceFrame: draft.referenceFrame,
            source: draft.source
        )
    }
}
