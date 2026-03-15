import SwiftUI

@main
struct RobotCopilotApp: App {

    @State private var appModel = AppModel()
    @State private var stylusModel = SpatialStylusModel()

    var body: some Scene {
        // Scene A: Main control window
        WindowGroup("Control", id: "control") {
            ControlWindowView()
                .environment(appModel)
                .environment(stylusModel)
                .task {
                    await appModel.bootstrapIfNeeded()
                }
        }
        .defaultSize(width: 700, height: 600)

        // Scene B: Volumetric robot view — opens alongside control window
        WindowGroup("Robot View", id: "robot-view") {
            RobotVolumeView()
                .environment(appModel)
                .environment(stylusModel)
        }
        .windowStyle(.volumetric)
        .defaultSize(width: 0.5, height: 0.5, depth: 0.5, in: .meters)

        // Scene C: ImmersiveSpace for spatial alignment (ARKit plane detection)
        ImmersiveSpace(id: appModel.immersiveSpaceID) {
            SpatialAlignmentView()
                .environment(appModel)
                .environment(stylusModel)
                .onAppear {
                    appModel.immersiveSpaceState = .open
                }
                .onDisappear {
                    appModel.immersiveSpaceState = .closed
                }
        }
        .immersionStyle(selection: .constant(.mixed), in: .mixed)
    }
}
