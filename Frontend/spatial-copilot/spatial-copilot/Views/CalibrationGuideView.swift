import SwiftUI
import RealityKit

/// In-scene onboarding guide for stylus-driven calibration and preview.
struct CalibrationGuideView: View {
    @Environment(SpatialSessionModel.self) private var spatial
    @Environment(RobotStateModel.self) private var state

    var body: some View {
        if let copy = stageCopy {
            VStack(spacing: 12) {
                Image(systemName: copy.symbol)
                    .font(.system(size: 48))
                    .foregroundStyle(copy.tint)
                    .symbolEffect(.pulse, isActive: true)

                Text(copy.title)
                    .font(.headline.weight(.bold))
                    .foregroundStyle(.primary)

                Text(copy.message)
                    .font(.subheadline.weight(.medium))
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                    .frame(maxWidth: 300)
            }
            .padding(24)
            .glassBackgroundEffect()
        }
    }

    private var stageCopy: (symbol: String, tint: Color, title: String, message: String)? {
        switch spatial.onboardingStage {
        case .needsStylus:
            (
                "pencil.slash",
                .orange,
                "Connect the spatial stylus",
                "Pair the stylus before calibration. The base and gripper must both be placed with the stylus."
            )
        case .needsRobotState:
            (
                "antenna.radiowaves.left.and.right",
                .orange,
                "Waiting for live robot pose",
                state.toolPoseStatusSummary
            )
        case .placingBase:
            (
                "scope",
                .cyan,
                "Place the robot base",
                "Move the stylus over a recognized table or floor surface. The orange marker will stick to the surface. Press the main stylus button to confirm the base."
            )
        case .placingGripper:
            (
                "move.3d",
                .cyan,
                "Place the gripper point",
                "Move the stylus to the robot gripper location in free space, then press the main stylus button to finish calibration."
            )
        case .previewing:
            nil
        case .confirming:
            (
                "checkmark.circle",
                .green,
                "Confirm the Cartesian move",
                "The green marker shows the resolved gripper target. Use the in-scene query or the stylus secondary button to cancel."
            )
        }
    }
}
