import SwiftUI
import RosClawBridge

/// Horizontal row of quick action capsule buttons — E-Stop, Home, Gripper, Spatial toggle.
struct QuickActionStrip: View {
    @Environment(CommandFlowModel.self) private var commandFlow
    @Environment(RobotConnectionModel.self) private var connection
    @Environment(RobotStateModel.self) private var state
    @Environment(SpatialSessionModel.self) private var spatial
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 10) {
                // E-Stop
                quickButton("E-STOP", icon: "stop.circle.fill", tint: .red) {
                    Task { await commandFlow.triggerEmergencyStop() }
                }

                // Home
                if connection.supportsHomeCommand {
                    quickButton("Home", icon: "house.fill", tint: .blue) {
                        Task {
                            if spatial.immersiveSpaceState == .open {
                                await commandFlow.executeQuickAction(.homeRobot)
                            } else {
                                await commandFlow.handleInput("home robot")
                            }
                        }
                    }
                }

                // Gripper toggle
                if connection.supportsCommand("openGripper") || connection.supportsCommand("closeGripper") {
                    let isOpen = state.robotState.gripperState == .open
                    quickButton(isOpen ? "Close" : "Open", icon: isOpen ? "hand.raised.slash.fill" : "hand.raised.fill", tint: .orange) {
                        Task {
                            if spatial.immersiveSpaceState == .open {
                                await commandFlow.executeQuickAction(isOpen ? .closeGripper : .openGripper)
                            } else {
                                await commandFlow.handleInput(isOpen ? "close gripper" : "open gripper")
                            }
                        }
                    }
                }

                // Look (perception snapshot)
                quickButton("Look", icon: "eye.fill", tint: .cyan) {
                    Task { await commandFlow.requestPerceptionSnapshot() }
                }

                // Spatial toggle
                quickButton(
                    spatial.immersiveSpaceState == .open ? "Exit Spatial" : "Spatial",
                    icon: spatial.immersiveSpaceState == .open ? "arrow.down.right.and.arrow.up.left" : "view.3d",
                    tint: .purple
                ) {
                    Task {
                        if spatial.immersiveSpaceState == .open {
                            await dismissImmersiveSpace()
                        } else {
                            await openImmersiveSpace(id: spatial.immersiveSpaceID)
                        }
                    }
                }
            }
            .padding(.horizontal, 16)
        }
    }

    private func quickButton(_ title: String, icon: String, tint: Color, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Label(title, systemImage: icon)
                .font(.subheadline.weight(.bold))
                .padding(.horizontal, 14)
                .padding(.vertical, 10)
        }
        .buttonStyle(.plain)
        .foregroundStyle(tint)
        .background(tint.opacity(0.15), in: Capsule())
        .hoverEffect()
    }
}
