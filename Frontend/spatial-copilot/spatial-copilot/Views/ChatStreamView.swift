import SwiftUI

/// Scrolling chat message list — newest messages at the bottom.
struct ChatStreamView: View {
    @Environment(CommandFlowModel.self) private var commandFlow
    @Environment(SpatialSessionModel.self) private var spatial

    var body: some View {
        ScrollViewReader { proxy in
            ScrollView {
                LazyVStack(spacing: 8) {
                    if commandFlow.transcript.isEmpty {
                        Text("Say or type a command to get started.")
                            .font(.body.weight(.medium))
                            .foregroundStyle(.secondary)
                            .padding(.top, 40)
                    } else {
                        ForEach(commandFlow.transcript) { entry in
                            if entry.kind == .parsedCommand &&
                                commandFlow.pendingPreview != nil &&
                                spatial.immersiveSpaceState != .open &&
                                entry.id == commandFlow.transcript.last(where: { $0.kind == .parsedCommand })?.id {
                                CommandPreviewCard()
                            } else {
                                ChatBubbleView(entry: entry)
                            }
                        }
                    }
                }
                .padding(.horizontal, 16)
                .padding(.vertical, 12)
            }
            .onChange(of: commandFlow.transcript.count) { _, _ in
                if let last = commandFlow.transcript.last {
                    withAnimation(.easeOut(duration: 0.2)) {
                        proxy.scrollTo(last.id, anchor: .bottom)
                    }
                }
            }
        }
    }
}
