import SwiftUI
import RosClawBridge

/// Chat bubble displaying a perception snapshot — image, objects, obstacles, and alerts.
struct PerceptionBubbleView: View {
    let entry: TranscriptEntry

    var body: some View {
        HStack {
            VStack(alignment: .leading, spacing: 10) {
                // Inline image
                if let snapshot = entry.perceptionSnapshot, !snapshot.imageData.isEmpty,
                   let uiImage = UIImage(data: snapshot.imageData) {
                    Image(uiImage: uiImage)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(maxWidth: 280, maxHeight: 200)
                        .clipShape(RoundedRectangle(cornerRadius: 12))
                }

                // Summary
                if !entry.text.isEmpty {
                    Text(entry.text)
                        .font(.body.weight(.medium))
                        .foregroundStyle(.primary)
                }

                // Objects
                if let snapshot = entry.perceptionSnapshot, !snapshot.objects.isEmpty {
                    VStack(alignment: .leading, spacing: 4) {
                        Label("Objects", systemImage: "cube.fill")
                            .font(.caption.weight(.bold))
                            .foregroundStyle(.cyan)
                        ForEach(snapshot.objects, id: \.label) { obj in
                            HStack(spacing: 6) {
                                Text(obj.label)
                                    .font(.caption.weight(.medium))
                                if obj.distanceM > 0 {
                                    Text(String(format: "%.2fm", obj.distanceM))
                                        .font(.caption2)
                                        .foregroundStyle(.secondary)
                                }
                                if !obj.position.isEmpty {
                                    Text(obj.position)
                                        .font(.caption2)
                                        .foregroundStyle(.secondary)
                                }
                            }
                        }
                    }
                }

                // Obstacles
                if let snapshot = entry.perceptionSnapshot, !snapshot.obstacles.isEmpty {
                    VStack(alignment: .leading, spacing: 4) {
                        Label("Obstacles", systemImage: "exclamationmark.triangle.fill")
                            .font(.caption.weight(.bold))
                            .foregroundStyle(.yellow)
                        ForEach(snapshot.obstacles, id: \.label) { obs in
                            HStack(spacing: 6) {
                                Text(obs.label)
                                    .font(.caption.weight(.medium))
                                if !obs.direction.isEmpty {
                                    Text(obs.direction)
                                        .font(.caption2)
                                        .foregroundStyle(.secondary)
                                }
                                if obs.distanceM > 0 {
                                    Text(String(format: "%.2fm", obs.distanceM))
                                        .font(.caption2)
                                        .foregroundStyle(.secondary)
                                }
                            }
                        }
                    }
                }

                // Alerts
                if let snapshot = entry.perceptionSnapshot, !snapshot.alerts.isEmpty {
                    VStack(alignment: .leading, spacing: 4) {
                        ForEach(snapshot.alerts, id: \.self) { alert in
                            Label(alert, systemImage: "bell.fill")
                                .font(.caption.weight(.medium))
                                .foregroundStyle(.red)
                        }
                    }
                }

                // Timestamp
                Text(entry.timestamp, style: .time)
                    .font(.caption2.weight(.semibold))
                    .foregroundStyle(.tertiary)
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 12)
            .background(Color.cyan.opacity(0.15), in: UnevenRoundedRectangle(
                topLeadingRadius: 16,
                bottomLeadingRadius: 4,
                bottomTrailingRadius: 16,
                topTrailingRadius: 16
            ))

            Spacer(minLength: 60)
        }
    }
}
