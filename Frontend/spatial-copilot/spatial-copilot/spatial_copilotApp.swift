import SwiftUI

@main
struct SpatialCopilotApp: App {

    @State private var coordinator = AppCoordinator()

    var body: some Scene {
        let immersivePanelActive = coordinator.spatial.immersiveSpaceState == .open

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
                    minWidth: immersivePanelActive ? 360 : 500,
                    idealWidth: immersivePanelActive ? 380 : 500,
                    maxWidth: immersivePanelActive ? 420 : .infinity,
                    minHeight: immersivePanelActive ? 220 : 700,
                    idealHeight: immersivePanelActive ? 240 : 700,
                    maxHeight: immersivePanelActive ? 260 : .infinity,
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
                    coordinator.commandFlow.enterImmersiveMode()
                    coordinator.spatial.immersiveSpaceState = .open
                }
                .onDisappear {
                    coordinator.spatial.immersiveSpaceState = .closed
                }
        }
        .immersionStyle(selection: .constant(.mixed), in: .mixed)
    }
}
