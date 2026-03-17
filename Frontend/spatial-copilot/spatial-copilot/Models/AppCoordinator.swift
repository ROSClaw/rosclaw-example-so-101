import SwiftUI
import RosClawBridge

/// Thin composition root that owns all domain models and wires their cross-references.
@MainActor
@Observable
final class AppCoordinator {

    let connection: RobotConnectionModel
    let state: RobotStateModel
    let commandFlow: CommandFlowModel
    let spatial: SpatialSessionModel
    let stylus: SpatialStylusModel

    init() {
        let connection = RobotConnectionModel()
        let state = RobotStateModel()
        let commandFlow = CommandFlowModel()
        let spatial = SpatialSessionModel()
        let stylus = SpatialStylusModel()

        // Wire cross-references
        connection.stateModel = state
        connection.spatialModel = spatial
        commandFlow.connectionModel = connection
        commandFlow.stateModel = state
        commandFlow.spatialModel = spatial
        spatial.connectionModel = connection
        spatial.stateModel = state
        spatial.commandFlowModel = commandFlow

        self.connection = connection
        self.state = state
        self.commandFlow = commandFlow
        self.spatial = spatial
        self.stylus = stylus
    }

    func bootstrapIfNeeded() async {
        await commandFlow.setupModelProvidersIfNeeded()
        await connection.bootstrapIfNeeded()
    }
}
