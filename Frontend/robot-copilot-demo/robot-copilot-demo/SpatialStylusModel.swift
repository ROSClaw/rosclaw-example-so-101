import ARKit
import GameController
import QuartzCore
import RealityKit
import Spatial
import SwiftUI

struct SpatialStylusButtonEvent: Sendable {
    enum Source: Sendable {
        case none
        case tip
        case primary
        case secondary
    }

    let source: Source
    let isPressed: Bool
    let pressure: Float
    let timestamp: TimeInterval
}

@MainActor
@Observable
final class SpatialStylusModel {
    private(set) var activeStylus: GCStylus?
    private(set) var accessoryTrackingAuthorizationStatus: ARKitSession.AuthorizationStatus?

    private let arKitSession = ARKitSession()
    private var accessoryTrackingProvider: AccessoryTrackingProvider?
    private var connectionObservers: [NSObjectProtocol] = []

    init() {
        Task { await monitorSessionEvents() }
        Task { await handleStylusConnections() }
    }

    var isConnected: Bool {
        activeStylus != nil
    }

    var isTracking: Bool {
        accessoryTrackingProvider?.state == .running
    }

    func getLatestButtonEvents() -> [SpatialStylusButtonEvent] {
        guard let stylusInput = activeStylus?.input else {
            return []
        }

        var events: [SpatialStylusButtonEvent] = []
        while let nextInputState = stylusInput.nextInputState() {
            guard let tipInput = nextInputState.buttons[.stylusTip]?.pressedInput,
                  let primaryInput = nextInputState.buttons[.stylusPrimaryButton]?.pressedInput,
                  let secondaryInput = nextInputState.buttons[.stylusSecondaryButton]?.pressedInput
            else {
                continue
            }

            let bySource: [(SpatialStylusButtonEvent.Source, Bool, Float)] = [
                (.tip, tipInput.isPressed, tipInput.value),
                (.primary, primaryInput.isPressed, primaryInput.value),
                (.secondary, secondaryInput.isPressed, secondaryInput.value),
            ]
            let isPressed = bySource.contains(where: { $0.1 })
            let strongest = bySource.max { $0.2 < $1.2 }
            let pressure = strongest?.2 ?? 0.0
            let source = pressure > 0 ? (strongest?.0 ?? .none) : .none

            events.append(
                SpatialStylusButtonEvent(
                    source: source,
                    isPressed: isPressed,
                    pressure: pressure,
                    timestamp: nextInputState.lastEventTimestamp
                )
            )
        }
        return events
    }

    func latestTipPosition(
        relativeTo entity: Entity,
        correction: ARKitCoordinateSpace.Correction = .rendered
    ) -> SIMD3<Float>? {
        guard let provider = accessoryTrackingProvider,
              provider.state == .running,
              let anchor = provider.latestAnchors.first
        else {
            return nil
        }
        return tipPosition(for: anchor, correction: correction, relativeTo: entity)
    }

    func predictedTipPosition(
        relativeTo entity: Entity,
        predictionHorizon: TimeInterval = (1.0 / 60.0),
        correction: ARKitCoordinateSpace.Correction = .rendered
    ) -> SIMD3<Float>? {
        guard let provider = accessoryTrackingProvider,
              provider.state == .running,
              let latestAnchor = provider.latestAnchors.first,
              let predicted = provider.predictAnchor(
                for: latestAnchor,
                at: CACurrentMediaTime() + predictionHorizon
              )
        else {
            return nil
        }
        return tipPosition(for: predicted, correction: correction, relativeTo: entity)
    }

    private func monitorSessionEvents() async {
        accessoryTrackingAuthorizationStatus =
            await arKitSession.queryAuthorization(for: [.accessoryTracking])[.accessoryTracking]
        for await event in arKitSession.events {
            switch event {
            case .authorizationChanged(.accessoryTracking, let status):
                accessoryTrackingAuthorizationStatus = status
                if status == .allowed, let activeStylus {
                    await runAccessoryTracking(for: activeStylus)
                }
            default:
                break
            }
        }
    }

    private func handleStylusConnections() async {
        if let stylus = GCStylus.styli.first(where: { $0.productCategory == GCProductCategorySpatialStylus }) {
            await setActiveStylus(stylus)
        }

        let connectObserver = NotificationCenter.default.addObserver(
            forName: NSNotification.Name.GCStylusDidConnect,
            object: nil,
            queue: nil
        ) { notification in
            guard let stylus = notification.object as? GCStylus,
                  stylus.productCategory == GCProductCategorySpatialStylus
            else {
                return
            }
            Task { @MainActor in
                await self.setActiveStylus(stylus)
            }
        }

        let disconnectObserver = NotificationCenter.default.addObserver(
            forName: NSNotification.Name.GCStylusDidDisconnect,
            object: nil,
            queue: nil
        ) { notification in
            guard let stylus = notification.object as? GCStylus else {
                return
            }
            Task { @MainActor in
                guard stylus == self.activeStylus else {
                    return
                }
                self.activeStylus = nil
                self.accessoryTrackingProvider = nil
                self.arKitSession.stop()
            }
        }

        connectionObservers = [connectObserver, disconnectObserver]
    }

    private func setActiveStylus(_ stylus: GCStylus) async {
        guard stylus != activeStylus else {
            return
        }
        activeStylus = stylus
        activeStylus?.input?.inputStateQueueDepth = 100
        await runAccessoryTracking(for: stylus)
    }

    private func runAccessoryTracking(for stylus: GCStylus) async {
        guard let accessory = try? await Accessory(device: stylus) else {
            return
        }
        let provider = AccessoryTrackingProvider(accessories: [accessory])
        accessoryTrackingProvider = provider
        try? await arKitSession.run([provider])
    }

    private func tipPosition(
        for anchor: AccessoryAnchor,
        correction: ARKitCoordinateSpace.Correction,
        relativeTo entity: Entity
    ) -> SIMD3<Float>? {
        guard let position = try? anchor.coordinateSpace(for: .aim, correction: correction)
            .convert(value: Point3DFloat.zero, to: entity)
        else {
            return nil
        }
        return [position.x, position.y, position.z]
    }
}
