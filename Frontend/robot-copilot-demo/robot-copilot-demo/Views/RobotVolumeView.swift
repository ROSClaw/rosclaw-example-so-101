import SwiftUI
import RealityKit
import RosClawBridge

struct RobotVolumeView: View {
    @Environment(AppModel.self) private var appModel
    @State private var robotEntity: Entity?
    @State private var selectedEntityName: String? = nil

    var body: some View {
        RealityView { content, attachments in
            // Build placeholder robot
            let robot = RobotEntityBuilder.buildRobotEntity()
            content.add(robot)
            robotEntity = robot

            // Workspace bounds
            let bounds = RobotEntityBuilder.buildWorkspaceBoundsEntity()
            content.add(bounds)

            // End-effector marker
            let marker = RobotEntityBuilder.buildEndEffectorMarker()
            content.add(marker)

            // Attach 2D status panel near end-effector
            if let statusPanel = attachments.entity(for: "status") {
                statusPanel.position = [0.12, 0.38, 0.16]
                content.add(statusPanel)
            }
        } update: { content, _ in
            // Update robot joint visuals when state changes
            updateRobotVisuals(in: content)
        } attachments: {
            Attachment(id: "status") {
                RobotStatusAttachmentView()
                    .environment(appModel)
            }
        }
        .gesture(
            TapGesture()
                .targetedToAnyEntity()
                .onEnded { value in
                    selectedEntityName = value.entity.name
                }
        )
        .overlay(alignment: .bottom) {
            if let name = selectedEntityName {
                Text("Selected: \(name)")
                    .font(.caption)
                    .padding(6)
                    .background(.regularMaterial, in: Capsule())
                    .padding(.bottom, 8)
            }
        }
    }

    private func updateRobotVisuals(in content: RealityViewContent) {
        // Visual fault indication — tint robot entity when faulted
        // Material update would go here when a real USDZ is used
        _ = robotEntity
    }
}
