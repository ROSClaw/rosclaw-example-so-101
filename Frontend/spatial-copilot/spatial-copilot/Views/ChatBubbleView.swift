import SwiftUI

/// Individual chat message bubble — right-aligned for user, left-aligned for system.
struct ChatBubbleView: View {
    let entry: TranscriptEntry

    var body: some View {
        HStack {
            if isUserMessage { Spacer(minLength: 60) }

            VStack(alignment: isUserMessage ? .trailing : .leading, spacing: 4) {
                Text(entry.text)
                    .font(.body.weight(.medium))
                    .foregroundStyle(textColor)

                Text(entry.timestamp, style: .time)
                    .font(.caption2.weight(.semibold))
                    .foregroundStyle(.tertiary)
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 12)
            .background(bubbleColor, in: bubbleShape)

            if !isUserMessage { Spacer(minLength: 60) }
        }
    }

    private var isUserMessage: Bool {
        entry.kind == .userInput
    }

    private var bubbleColor: Color {
        switch entry.kind {
        case .userInput: return .blue.opacity(0.3)
        case .error: return .red.opacity(0.2)
        case .parsedCommand: return .purple.opacity(0.15)
        case .feedback: return .green.opacity(0.15)
        case .system: return .gray.opacity(0.15)
        }
    }

    private var textColor: Color {
        switch entry.kind {
        case .error: return .red
        case .feedback: return .green
        default: return .primary
        }
    }

    private var bubbleShape: UnevenRoundedRectangle {
        if isUserMessage {
            return UnevenRoundedRectangle(topLeadingRadius: 16, bottomLeadingRadius: 16, bottomTrailingRadius: 4, topTrailingRadius: 16)
        } else {
            return UnevenRoundedRectangle(topLeadingRadius: 16, bottomLeadingRadius: 4, bottomTrailingRadius: 16, topTrailingRadius: 16)
        }
    }
}
