import SwiftUI

@main
struct SpatialCopilotApp: App {

    @State private var coordinator = AppCoordinator()

    var body: some Scene {
        // Compact floating copilot panel
        WindowGroup("Copilot", id: "copilot-panel") {
            CopilotPanelView()
                .environment(coordinator)
                .environment(coordinator.connection)
                .environment(coordinator.state)
                .environment(coordinator.commandFlow)
                .environment(coordinator.spatial)
                .environment(coordinator.stylus)
                .frame(
                    minWidth: 500,
                    maxWidth: .infinity,
                    minHeight: 700,
                    maxHeight: .infinity,
                    alignment: .topLeading
                )
                .task {
                    await coordinator.bootstrapIfNeeded()
                }
        }
        .defaultSize(width: 500, height: 700)
        .windowResizability(.contentMinSize)

        // Primary immersive space
        ImmersiveSpace(id: coordinator.spatial.immersiveSpaceID) {
            SpatialWorkspaceView()
                .environment(coordinator)
                .environment(coordinator.connection)
                .environment(coordinator.state)
                .environment(coordinator.commandFlow)
                .environment(coordinator.spatial)
                .environment(coordinator.stylus)
                .onAppear {
                    coordinator.spatial.immersiveSpaceState = .open
                }
                .onDisappear {
                    coordinator.spatial.immersiveSpaceState = .closed
                }
        }
        .immersionStyle(selection: .constant(.mixed), in: .mixed)
    }
}
