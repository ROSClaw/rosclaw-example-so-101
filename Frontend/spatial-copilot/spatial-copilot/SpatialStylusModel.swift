import ARKit
import GameController
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
    private(set) var trackingAccessory: Accessory?
    private(set) var connectionGeneration: UInt = 0

    private var connectionObservers: [NSObjectProtocol] = []
    private var tipPressed = false
    private var primaryPressed = false
    private var secondaryPressed = false

    init() {
        Task { await handleStylusConnections() }
    }

    var isConnected: Bool {
        activeStylus != nil
    }

    var hasTrackingAccessory: Bool {
        trackingAccessory != nil
    }

    var isTracking: Bool {
        hasTrackingAccessory
    }

    func getLatestButtonEvents() -> [SpatialStylusButtonEvent] {
        guard let stylusInput = activeStylus?.input else {
            return []
        }

        var events: [SpatialStylusButtonEvent] = []
        while let nextInputState = stylusInput.nextInputState() {
            let tipButton = nextInputState.buttons[.stylusTip]
            let primaryButton = nextInputState.buttons[.stylusPrimaryButton]
            let secondaryButton = nextInputState.buttons[.stylusSecondaryButton]
            let samples: [(SpatialStylusButtonEvent.Source, Bool, Float)] = [
                (
                    .tip,
                    tipButton?.pressedInput.isPressed ?? false,
                    tipButton?.pressedInput.value ?? 0
                ),
                (
                    .primary,
                    primaryButton?.pressedInput.isPressed ?? false,
                    primaryButton?.pressedInput.value ?? 0
                ),
                (
                    .secondary,
                    secondaryButton?.pressedInput.isPressed ?? false,
                    secondaryButton?.pressedInput.value ?? 0
                ),
            ]

            for sample in samples {
                if shouldEmit(sample) {
                    events.append(
                        SpatialStylusButtonEvent(
                            source: sample.0,
                            isPressed: sample.1,
                            pressure: sample.2,
                            timestamp: nextInputState.lastEventTimestamp
                        )
                    )
                    record(sample)
                }
            }
        }
        return events
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
                self.trackingAccessory = nil
                self.tipPressed = false
                self.primaryPressed = false
                self.secondaryPressed = false
                self.connectionGeneration &+= 1
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
        trackingAccessory = nil
        tipPressed = false
        primaryPressed = false
        secondaryPressed = false
        connectionGeneration &+= 1
        if let accessory = try? await Accessory(device: stylus), activeStylus == stylus {
            trackingAccessory = accessory
            connectionGeneration &+= 1
        }
    }

    private func shouldEmit(_ sample: (SpatialStylusButtonEvent.Source, Bool, Float)) -> Bool {
        switch sample.0 {
        case .tip:
            sample.1 != tipPressed
        case .primary:
            sample.1 != primaryPressed
        case .secondary:
            sample.1 != secondaryPressed
        case .none:
            false
        }
    }

    private func record(_ sample: (SpatialStylusButtonEvent.Source, Bool, Float)) {
        switch sample.0 {
        case .tip:
            tipPressed = sample.1
        case .primary:
            primaryPressed = sample.1
        case .secondary:
            secondaryPressed = sample.1
        case .none:
            break
        }
    }
}
