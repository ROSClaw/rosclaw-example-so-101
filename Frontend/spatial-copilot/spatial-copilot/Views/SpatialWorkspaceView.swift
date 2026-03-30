import ARKit
import QuartzCore
import RealityKit
import RosClawBridge
import Spatial
import SwiftUI
internal import UIKit

/// Primary immersive space for stylus-driven onboarding, robot preview, and Cartesian confirmation.
struct SpatialWorkspaceView: View {
    @Environment(RobotConnectionModel.self) private var connection
    @Environment(RobotStateModel.self) private var state
    @Environment(SpatialSessionModel.self) private var spatial
    @Environment(CommandFlowModel.self) private var commandFlow
    @Environment(SpatialStylusModel.self) private var stylusModel

    private let arkitSession = ARKitSession()
    private let worldTracking = WorldTrackingProvider()
    private let planeDetection = PlaneDetectionProvider(alignments: [.horizontal])
    private let jointStreamingPolicy = SpatialJointStreamingPolicy()
    private let surfaceProjectionMaxDistance: Float = 0.35
    private let stylusRayLiftMeters: Float = 0.12

    @State private var sceneRoot: Entity?
    @State private var robotModel: RobotModelEntity?
    @State private var workspaceVolume: Entity?
    @State private var lastWorkspaceBounds: WorkspaceBounds?
    @State private var planeVisualizationRoot: Entity?
    @State private var planeVisualizations: [UUID: ModelEntity] = [:]
    @State private var surfaceOverlay: Entity?
    @State private var placementMarker: ModelEntity?
    @State private var targetMarker: ModelEntity?
    @State private var statusBadge: Entity?
    @State private var confirmPanel: Entity?
    @State private var guidePanel: Entity?
    @State private var accessoryTrackingProvider: AccessoryTrackingProvider?
    @State private var planeMeshes: [UUID: SpatialPlaneMesh] = [:]
    @State private var currentSurfacePlacement: SurfacePlacement?
    @State private var currentTipWorldPoint: SIMD3<Float>?
    @State private var lastJointStreamingState: SpatialJointStreamingState?
    @State private var jointStreamRequestInFlight = false
    @State private var jointStreamingPausedForInvalidTarget = false
    @State private var surfaceHintAnnounced = false
    @State private var sceneUpdateSubscription: EventSubscription?

    var body: some View {
        RealityView { content, attachments in
            let root = Entity()
            content.add(root)
            sceneRoot = root

            let planeVisualizationContainer = Entity()
            planeVisualizationContainer.isEnabled = false
            root.addChild(planeVisualizationContainer)
            planeVisualizationRoot = planeVisualizationContainer

            let overlay = buildSurfaceOverlay()
            overlay.isEnabled = false
            root.addChild(overlay)
            surfaceOverlay = overlay

            let placement = buildPlacementMarker()
            placement.isEnabled = false
            root.addChild(placement)
            placementMarker = placement

            let target = buildTargetMarker()
            target.isEnabled = false
            root.addChild(target)
            targetMarker = target

            if let badge = attachments.entity(for: "status-badge") {
                badge.isEnabled = true
                root.addChild(badge)
                statusBadge = badge
            }
            if let panel = attachments.entity(for: "confirm-goal") {
                panel.isEnabled = false
                panel.components.set(BillboardComponent())
                root.addChild(panel)
                confirmPanel = panel
            }
            if let guide = attachments.entity(for: "calibration-guide") {
                guide.isEnabled = true
                guide.components.set(BillboardComponent())
                root.addChild(guide)
                guidePanel = guide
            }

            Task { @MainActor in
                let model = await RobotModelEntity()
                model.rootEntity.isEnabled = false
                root.addChild(model.rootEntity)
                robotModel = model
                rebuildWorkspaceVolumeIfNeeded(root: root)
            }

            sceneUpdateSubscription = content.subscribe(to: SceneEvents.Update.self) { _ in
                Task { @MainActor in
                    guard let root = sceneRoot else { return }
                    processSceneUpdate(relativeTo: root)
                }
            }
        } update: { _, _ in
            syncRobotPresentation()
        } attachments: {
            Attachment(id: "status-badge") {
                RobotStatusBadge()
                    .environment(state)
                    .environment(commandFlow)
            }
            Attachment(id: "confirm-goal") {
                CartesianConfirmPanel(
                    position: spatial.pendingSpatialRobotGoal,
                    onConfirm: { pos in
                        Task { await sendGoal(pos) }
                    },
                    onCancel: { cancelConfirmation() }
                )
            }
            Attachment(id: "calibration-guide") {
                CalibrationGuideView()
                    .environment(spatial)
                    .environment(state)
            }
        }
        .task(id: stylusModel.connectionGeneration) {
            await configureARKitSession()
        }
        .task {
            await processPlaneUpdates()
        }
        .task {
            await processWorldAnchorUpdates()
        }
        .task {
            await maintainRobotPoseRefreshLoop()
        }
        .onDisappear {
            sceneUpdateSubscription?.cancel()
            sceneUpdateSubscription = nil
            planeMeshes.removeAll()
            planeVisualizations.removeAll()
            lastJointStreamingState = nil
            jointStreamingPausedForInvalidTarget = false
            arkitSession.stop()
            Task {
                await connection.stopJointPreviewStreaming()
                await removeCalibrationWorldAnchor()
            }
        }
    }

    // MARK: - Scene update

    private func processSceneUpdate(relativeTo root: Entity) {
        spatial.synchronizeRequirements(
            stylusConnected: stylusModel.hasTrackingAccessory,
            hasRobotToolPose: state.robotState.toolPose?.position != nil
        )

        rebuildWorkspaceVolumeIfNeeded(root: root)

        if spatial.onboardingStage != .confirming {
            if spatial.controlMode == .simToRealHolding {
                currentTipWorldPoint = stylusTipPosition(relativeTo: root, predicted: false)
            } else {
                currentTipWorldPoint =
                    stylusTipPosition(relativeTo: root, predicted: true) ??
                    stylusTipPosition(relativeTo: root, predicted: false)
            }
        }

        renderCurrentStage(relativeTo: root)
        handleButtonEvents()
        renderCurrentStage(relativeTo: root)
        updateSimToRealStreaming()
        updateStatusBadgePosition()
        updateGuidePanelPosition()
    }

    private func renderCurrentStage(relativeTo root: Entity) {
        switch spatial.onboardingStage {
        case .needsStylus, .needsRobotState:
            planeVisualizationRoot?.isEnabled = false
            currentSurfacePlacement = nil
            surfaceOverlay?.isEnabled = false
            placementMarker?.isEnabled = false
            confirmPanel?.isEnabled = false
            if !spatial.isCalibrated {
                targetMarker?.isEnabled = false
                spatial.setPreviewState(nil)
                setAppPhase(.idle)
            }
        case .placingBase:
            planeVisualizationRoot?.isEnabled = true
            spatial.setPreviewState(nil)
            setAppPhase(.idle)
            targetMarker?.isEnabled = false
            confirmPanel?.isEnabled = false
            renderBasePlacement()
        case .placingGripper:
            planeVisualizationRoot?.isEnabled = false
            setAppPhase(.idle)
            targetMarker?.isEnabled = false
            confirmPanel?.isEnabled = false
            renderGripperPlacement()
        case .previewing:
            planeVisualizationRoot?.isEnabled = false
            setAppPhase(spatial.controlMode == .simToRealHolding ? .executing : .previewing)
            confirmPanel?.isEnabled = false
            renderRobotPreview()
        case .confirming:
            planeVisualizationRoot?.isEnabled = false
            setAppPhase(.awaitingConfirmation)
            renderConfirmation()
        }

        syncRobotPresentation()
    }

    private func renderBasePlacement() {
        guard let tip = currentTipWorldPoint else {
            currentSurfacePlacement = nil
            surfaceOverlay?.isEnabled = false
            placementMarker?.isEnabled = false
            return
        }

        guard let surfacePlacement = resolvedSurfacePlacement(for: tip) else {
            currentSurfacePlacement = nil
            setMarkerColor(placementMarker, isValid: false)
            placementMarker?.position = tip
            placementMarker?.isEnabled = true
            surfaceOverlay?.isEnabled = false
            return
        }

        currentSurfacePlacement = surfacePlacement
        setMarkerColor(placementMarker, isValid: true)
        placementMarker?.position = surfacePlacement.worldPoint + (surfacePlacement.surfaceNormal * 0.02)
        placementMarker?.isEnabled = true
        surfaceOverlay?.transform = Transform(
            scale: SIMD3<Float>(repeating: 1),
            rotation: rotationAligningUp(to: surfacePlacement.surfaceNormal),
            translation: surfacePlacement.worldPoint
        )
        surfaceOverlay?.isEnabled = true
    }

    private func renderGripperPlacement() {
        currentSurfacePlacement = nil
        surfaceOverlay?.isEnabled = false
        guard let tip = currentTipWorldPoint,
              let pendingBasePlacement = spatial.pendingBasePlacement else {
            placementMarker?.isEnabled = false
            spatial.setPreviewState(nil)
            return
        }

        let previewState = previewState(
            for: tip,
            robotToWorldTransform: pendingBasePlacement.robotToWorldTransform
        )
        spatial.setPreviewState(previewState)

        setMarkerColor(placementMarker, isValid: previewState.isValid)
        placementMarker?.position = tip
        placementMarker?.isEnabled = true
    }

    private func renderRobotPreview() {
        surfaceOverlay?.isEnabled = false
        placementMarker?.isEnabled = false

        guard let calibration = spatial.calibrationPayload,
              let tip = currentTipWorldPoint else {
            targetMarker?.isEnabled = false
            spatial.setPreviewState(nil)
            return
        }

        let previewState = previewState(
            for: tip,
            robotToWorldTransform: calibration.robotToWorldTransform
        )
        spatial.setPreviewState(previewState)

        setMarkerColor(targetMarker, isValid: previewState.isValid)
        targetMarker?.position = previewState.targetWorldPoint
        targetMarker?.isEnabled = true
    }

    private func renderConfirmation() {
        surfaceOverlay?.isEnabled = false
        placementMarker?.isEnabled = false
        confirmPanel?.isEnabled = spatial.pendingSpatialRobotGoal != nil
        if let worldPoint = spatial.pendingGoalWorldPoint {
            setMarkerColor(targetMarker, isValid: true)
            targetMarker?.position = worldPoint
            targetMarker?.isEnabled = true
            confirmPanel?.position = worldPoint + SIMD3<Float>(0, 0.13, 0)
        }
    }

    // MARK: - Input handling

    private func handleButtonEvents() {
        for event in stylusModel.getLatestButtonEvents() {
            switch event.source {
            case .primary:
                if event.isPressed {
                    Task { @MainActor in await handlePrimaryPress() }
                } else {
                    Task { @MainActor in await handlePrimaryRelease() }
                }
            case .secondary:
                if event.isPressed {
                    handleSecondaryPress()
                }
            case .tip:
                if event.isPressed {
                    handleTipPress()
                }
            case .none:
                break
            }
        }
    }

    private func handlePrimaryPress() async {
        switch spatial.onboardingStage {
        case .needsStylus:
            commandFlow.appendTranscript(.error, "Connect the spatial stylus before starting immersive calibration.")
        case .needsRobotState:
            commandFlow.appendTranscript(.error, "Live robot gripper pose is still unavailable.")
        case .placingBase:
            guard let sampledTip = await stabilizedStylusTipPosition(),
                  let surfacePlacement = resolvedSurfacePlacement(for: sampledTip) else {
                commandFlow.appendTranscript(.error, "Move the stylus onto a recognized surface before confirming the robot base.")
                return
            }
            let baseTransform = SpatialTargetingMath.basePlacementTransform(
                baseWorldPoint: surfacePlacement.worldPoint,
                surfaceNormal: surfacePlacement.surfaceNormal
            )
            spatial.registerBasePlacement(
                surfaceAnchorID: surfacePlacement.anchorID,
                surfaceNormal: surfacePlacement.surfaceNormal,
                worldPoint: surfacePlacement.worldPoint,
                robotToWorldTransform: baseTransform
            )
            commandFlow.appendTranscript(.system, "Robot base placed. Move the stylus to the real gripper so the USDZ arm can be aligned.")
        case .placingGripper:
            guard let sampledTip = await stabilizedStylusTipPosition(),
                  let pendingBasePlacement = spatial.pendingBasePlacement else {
                commandFlow.appendTranscript(.error, "The virtual robot could not be calibrated from the sampled base and gripper points.")
                return
            }
            guard let robotToWorldTransform = resolvedCalibrationTransform(
                for: sampledTip,
                pendingBasePlacement: pendingBasePlacement
            ) else {
                commandFlow.appendTranscript(.error, "The virtual robot could not be calibrated from the sampled base and gripper points.")
                return
            }
            let preview = previewState(
                for: sampledTip,
                robotToWorldTransform: robotToWorldTransform
            )
            guard preview.isValid else {
                commandFlow.appendTranscript(.error, invalidTargetMessage(for: preview.invalidReason, stage: .placingGripper))
                return
            }
            let updatedAnchor: WorldAnchor?
            do {
                updatedAnchor = try await createWorldAnchor(transform: robotToWorldTransform)
            } catch {
                updatedAnchor = nil
                commandFlow.appendTranscript(
                    .system,
                    "Calibration completed in the current world frame. Persistent world anchoring is unavailable right now because \(error.localizedDescription)."
                )
            }
            guard spatial.completeCalibration(
                worldAnchorID: updatedAnchor?.id,
                gripperWorldPoint: sampledTip,
                robotToWorldTransform: robotToWorldTransform
            ) else {
                commandFlow.appendTranscript(.error, "The virtual robot could not be calibrated from the sampled base and gripper points.")
                return
            }
            commandFlow.appendTranscript(.system, "Calibration complete. The virtual gripper now follows the stylus preview; hold the main stylus button to drive the real robot from the preview.")
        case .previewing:
            guard let preview = spatial.previewState else {
                commandFlow.appendTranscript(.error, "The stylus target could not be resolved.")
                return
            }
            guard preview.isValid else {
                commandFlow.appendTranscript(.error, invalidTargetMessage(for: preview.invalidReason, stage: .previewing))
                return
            }
            guard jointStreamingPolicy.canStream(preview: preview) else {
                commandFlow.appendTranscript(.error, "Sim-to-real hold is blocked because the preview IK error is too large.")
                return
            }
            spatial.beginSimToRealHolding()
            commandFlow.appendTranscript(.system, "Sim-to-real hold active. The preview joints are driving the real arm until you release the main button.")
        case .confirming:
            break
        }
    }

    private func handlePrimaryRelease() async {
        guard spatial.controlMode == .simToRealHolding else { return }
        spatial.endSimToRealHolding()
        lastJointStreamingState = nil
        jointStreamingPausedForInvalidTarget = false
        await connection.stopJointPreviewStreaming()
        commandFlow.appendTranscript(.system, "Sim-to-real hold released. The virtual preview is active again without moving the real robot.")
    }

    private func handleTipPress() {
        guard spatial.onboardingStage == .previewing,
              spatial.controlMode != .simToRealHolding else {
            return
        }
        guard let preview = spatial.previewState else {
            commandFlow.appendTranscript(.error, "The stylus target could not be resolved.")
            return
        }
        guard preview.isValid else {
            commandFlow.appendTranscript(.error, invalidTargetMessage(for: preview.invalidReason, stage: .previewing))
            return
        }
        spatial.setPendingSpatialGoal(preview.requestedRobotPoint, worldPoint: preview.targetWorldPoint)
    }

    private func handleSecondaryPress() {
        if spatial.controlMode == .simToRealHolding {
            Task { @MainActor in await handlePrimaryRelease() }
            return
        }
        if spatial.onboardingStage == .confirming {
            cancelConfirmation()
        }
    }

    private func cancelConfirmation() {
        spatial.clearPendingSpatialGoal()
        confirmPanel?.isEnabled = false
        if spatial.isCalibrated {
            setAppPhase(.previewing)
        } else {
            setAppPhase(.idle)
        }
    }

    // MARK: - ARKit / tracking

    private func configureARKitSession() async {
        guard WorldTrackingProvider.isSupported && PlaneDetectionProvider.isSupported else {
            commandFlow.appendTranscript(.error, "World tracking and plane detection are unavailable on this device.")
            return
        }

        var providers: [any DataProvider] = [worldTracking, planeDetection]
        var requestedAuthorizations: [ARKitSession.AuthorizationType] = [.worldSensing]

        if let accessory = stylusModel.trackingAccessory, AccessoryTrackingProvider.isSupported {
            let provider = AccessoryTrackingProvider(accessories: [accessory])
            accessoryTrackingProvider = provider
            providers.append(provider)
            requestedAuthorizations.append(.accessoryTracking)
        } else {
            accessoryTrackingProvider = nil
        }

        let authorization = await arkitSession.requestAuthorization(for: requestedAuthorizations)
        if authorization[.worldSensing] != .allowed {
            commandFlow.appendTranscript(.error, "World sensing permission is required for immersive calibration.")
            return
        }
        if requestedAuthorizations.contains(.accessoryTracking),
           authorization[.accessoryTracking] != .allowed {
            commandFlow.appendTranscript(.error, "Accessory tracking permission is required to use the spatial stylus in the immersive space.")
            return
        }

        do {
            try await arkitSession.run(providers)
        } catch {
            commandFlow.appendTranscript(.error, "ARKit session failed: \(error.localizedDescription)")
        }
    }

    private func processPlaneUpdates() async {
        for await update in planeDetection.anchorUpdates {
            let anchor = update.anchor
            if isEligible(anchor), !surfaceHintAnnounced {
                surfaceHintAnnounced = true
                commandFlow.appendTranscript(.system, "Surface recognized. Use the stylus to place the base on the surface, then sample the gripper in free space.")
            }

            switch update.event {
            case .added, .updated:
                if isEligible(anchor) {
                    planeMeshes[anchor.id] = SpatialPlaneMesh(anchor: anchor)
                    await upsertPlaneVisualization(for: anchor)
                } else {
                    planeMeshes.removeValue(forKey: anchor.id)
                    await removePlaneVisualization(for: anchor.id)
                }
            case .removed:
                planeMeshes.removeValue(forKey: anchor.id)
                await removePlaneVisualization(for: anchor.id)
            }
        }
    }

    private func processWorldAnchorUpdates() async {
        for await update in worldTracking.anchorUpdates {
            let anchor = update.anchor
            switch update.event {
            case .added, .updated:
                spatial.updateWorldAnchorTransform(
                    anchorID: anchor.id,
                    transform: anchor.originFromAnchorTransform
                )
            case .removed:
                continue
            }
        }
    }

    private func maintainRobotPoseRefreshLoop() async {
        while !Task.isCancelled {
            if (spatial.onboardingStage == .needsRobotState || spatial.controlMode == .realToSim),
               state.robotState.toolPose?.position == nil,
               connection.remoteBootState == .ready {
                await connection.refreshRobotSnapshot()
            }

            do {
                try await Task.sleep(for: .seconds(2))
            } catch {
                return
            }
        }
    }

    private func stylusTipPosition(relativeTo root: Entity, predicted: Bool) -> SIMD3<Float>? {
        guard let provider = accessoryTrackingProvider,
              provider.state == .running,
              let latestAnchor = provider.latestAnchors.first else {
            return nil
        }

        let anchor: AccessoryAnchor
        if predicted,
           let prediction = provider.predictAnchor(
               for: latestAnchor,
               at: CACurrentMediaTime() + (1.0 / 60.0)
           ) {
            anchor = prediction
        } else {
            anchor = latestAnchor
        }

        guard anchor.isTracked,
              let point = try? anchor.coordinateSpace(for: .aim, correction: .rendered)
            .convert(value: Point3DFloat.zero, to: root) else {
            return nil
        }
        return SIMD3<Float>(point.x, point.y, point.z)
    }

    private func stabilizedStylusTipPosition(
        sampleCount: Int = 6,
        intervalMilliseconds: Int = 20
    ) async -> SIMD3<Float>? {
        guard let root = sceneRoot else { return currentTipWorldPoint }
        var samples: [SIMD3<Float>] = []
        for index in 0..<sampleCount {
            if let sample = stylusTipPosition(relativeTo: root, predicted: false) {
                samples.append(sample)
            }
            if index + 1 < sampleCount {
                try? await Task.sleep(for: .milliseconds(intervalMilliseconds))
            }
        }

        guard !samples.isEmpty else { return currentTipWorldPoint }
        let sum = samples.reduce(SIMD3<Float>.zero, +)
        return sum / Float(samples.count)
    }

    // MARK: - Presentation helpers

    private func syncRobotPresentation() {
        let activeRobotTransform: simd_float4x4?
        switch spatial.onboardingStage {
        case .placingGripper:
            activeRobotTransform = spatial.pendingBasePlacement?.robotToWorldTransform
        default:
            activeRobotTransform = spatial.calibrationPayload?.robotToWorldTransform
        }

        if let activeRobotTransform {
            robotModel?.setCalibrationTransform(activeRobotTransform)
            robotModel?.rootEntity.isEnabled = true
            workspaceVolume?.transform = Transform(matrix: activeRobotTransform)
            workspaceVolume?.isEnabled = spatial.onboardingStage == .placingGripper || spatial.calibrationPayload != nil
        } else {
            robotModel?.rootEntity.isEnabled = false
            workspaceVolume?.isEnabled = false
        }

        let shouldApplyPreviewJoints =
            spatial.onboardingStage == .placingGripper ||
            spatial.onboardingStage == .previewing ||
            spatial.controlMode == .simToRealHolding

        if shouldApplyPreviewJoints,
           let preview = spatial.previewState {
            robotModel?.applyPreviewJoints(preview.previewJointPositions)
        } else {
            robotModel?.applyPreviewJoints(nil)
            robotModel?.updateJoints(from: state.robotState)
        }
        robotModel?.updateGripper(openFraction: state.robotState.gripperOpenFraction)
        robotModel?.setFaulted(state.robotState.mode == .faulted)
    }

    private func rebuildWorkspaceVolumeIfNeeded(root: Entity) {
        guard lastWorkspaceBounds != connection.workspaceBounds else { return }
        workspaceVolume?.removeFromParent()
        let volume = buildWorkspaceVolume(bounds: connection.workspaceBounds)
        volume.isEnabled = spatial.calibrationPayload != nil || spatial.pendingBasePlacement != nil
        root.addChild(volume)
        workspaceVolume = volume
        lastWorkspaceBounds = connection.workspaceBounds
    }

    private func updateSimToRealStreaming() {
        guard spatial.controlMode == .simToRealHolding else {
            lastJointStreamingState = nil
            jointStreamingPausedForInvalidTarget = false
            return
        }
        guard let preview = spatial.previewState else {
            pauseJointStreamingForInvalidTarget(reason: "Sim-to-real hold is paused because the preview target is unavailable.")
            lastJointStreamingState = nil
            return
        }
        guard jointStreamingPolicy.canStream(preview: preview) else {
            pauseJointStreamingForInvalidTarget(reason: invalidTargetMessage(for: preview.invalidReason, stage: .previewing))
            lastJointStreamingState = nil
            return
        }
        if jointStreamingPausedForInvalidTarget {
            jointStreamingPausedForInvalidTarget = false
            lastJointStreamingState = nil
            commandFlow.appendTranscript(.system, "Sim-to-real hold resumed. The target is back inside the safe zone.")
        }
        guard !jointStreamRequestInFlight else { return }

        let now = CACurrentMediaTime()
        guard jointStreamingPolicy.shouldStream(
            preview: preview,
            previous: lastJointStreamingState,
            now: now
        ) else {
            return
        }

        jointStreamRequestInFlight = true
        lastJointStreamingState = SpatialJointStreamingState(
            jointPositions: preview.previewJointPositions,
            endEffectorWorldPoint: preview.previewEndEffectorWorldPoint,
            sentAt: now
        )

        Task { @MainActor in
            defer { jointStreamRequestInFlight = false }
            do {
                try await connection.streamJointPreview(
                    preview.previewJointPositions,
                    durationSec: 0.12
                )
            } catch {
                spatial.endSimToRealHolding()
                lastJointStreamingState = nil
                jointStreamingPausedForInvalidTarget = false
                await connection.stopJointPreviewStreaming()
                commandFlow.appendTranscript(.error, "Live sim-to-real streaming failed: \(error.localizedDescription)")
            }
        }
    }

    private func updateStatusBadgePosition() {
        guard let badge = statusBadge else { return }
        if spatial.controlMode == .simToRealHolding,
           let previewPoint = spatial.previewState?.previewEndEffectorWorldPoint {
            badge.position = previewPoint + SIMD3<Float>(0.06, 0.08, 0.04)
            return
        }
        if let pendingWorldPoint = spatial.pendingGoalWorldPoint {
            badge.position = pendingWorldPoint + SIMD3<Float>(0.06, 0.08, 0.04)
            return
        }
        if let calibration = spatial.calibrationPayload {
            badge.position = calibration.baseWorldPoint + SIMD3<Float>(0.16, 0.24, 0.08)
            return
        }
        if let pendingBaseWorldPoint = spatial.pendingBaseWorldPoint {
            badge.position = pendingBaseWorldPoint + SIMD3<Float>(0.12, 0.16, 0.06)
            return
        }
        badge.position = SIMD3<Float>(0.0, 1.1, -0.6)
    }

    private func updateGuidePanelPosition() {
        guard let guide = guidePanel else { return }
        switch spatial.onboardingStage {
        case .placingBase:
            guide.position = (currentSurfacePlacement?.worldPoint ?? SIMD3<Float>(0, 1.1, -0.6)) + SIMD3<Float>(0, 0.1, 0)
            guide.isEnabled = true
        case .placingGripper:
            guide.position = (currentTipWorldPoint ?? spatial.pendingBaseWorldPoint ?? SIMD3<Float>(0, 1.1, -0.6)) + SIMD3<Float>(0, 0.1, 0)
            guide.isEnabled = true
        case .needsStylus, .needsRobotState:
            guide.position = (spatial.pendingBaseWorldPoint ?? SIMD3<Float>(0, 1.1, -0.6)) + SIMD3<Float>(0, 0.1, 0)
            guide.isEnabled = true
        case .confirming, .previewing:
            guide.isEnabled = false
        }
    }

    private func setAppPhase(_ phase: AppPhase) {
        if commandFlow.appPhase != phase {
            commandFlow.appPhase = phase
        }
    }

    // MARK: - Spatial calculations

    private func resolvedSurfacePlacement(for point: SIMD3<Float>) -> SurfacePlacement? {
        let raycastCandidate = resolvedSurfacePlacementFromRaycast(for: point)
        let projectionCandidates = SpatialPlaneProjector.project(
            point: point,
            onto: Array(planeMeshes.values),
            maxDistance: surfaceProjectionMaxDistance
        )

        return SpatialPlaneProjector
            .resolvePreferredCandidate(
                raycastCandidate: raycastCandidate,
                projectionCandidates: projectionCandidates
            )
            .map { candidate in
                SurfacePlacement(
                    anchorID: candidate.anchorID,
                    worldPoint: candidate.worldPoint,
                    surfaceNormal: candidate.surfaceNormal,
                    distanceToTip: candidate.distanceToPoint,
                    score: candidate.score,
                    resolutionMethod: candidate.method,
                    comparisonDeltaMeters: candidate.comparisonDeltaMeters
                )
            }
    }

    private func resolvedSurfacePlacementFromRaycast(for point: SIMD3<Float>) -> SpatialPlaneProjector.RaycastCandidate? {
        guard let root = sceneRoot,
              let raycastResult = root.scene?.raycast(
                  origin: point + SIMD3<Float>(0, stylusRayLiftMeters, 0),
                  direction: SIMD3<Float>(0, -1, 0),
                  length: surfaceProjectionMaxDistance + stylusRayLiftMeters,
                  query: .nearest,
                  mask: PlaneAnchor.horizontalCollisionGroup
              ).first,
              let anchorID = raycastResult.entity.components[PlaneAnchorIDComponent.self]?.anchorID,
              let planeMesh = planeMeshes[anchorID]
        else {
            return nil
        }

        let distanceToTip = simd_length(raycastResult.position - point)
        guard distanceToTip <= surfaceProjectionMaxDistance else {
            return nil
        }

        return SpatialPlaneProjector.RaycastCandidate(
            anchorID: anchorID,
            worldPoint: raycastResult.position,
            surfaceNormal: planeMesh.surfaceNormal,
            distanceToPoint: distanceToTip,
            surfaceArea: planeMesh.areaEstimate,
            classificationBias: planeMesh.classificationBias
        )
    }

    private func resolvedCalibrationTransform(
        for gripperWorldPoint: SIMD3<Float>,
        pendingBasePlacement: SpatialSessionModel.PendingBasePlacement
    ) -> simd_float4x4? {
        guard let toolPosePosition = state.robotState.toolPose?.position else {
            return nil
        }

        let toolVector = SIMD3<Float>(
            Float(toolPosePosition.x),
            Float(toolPosePosition.y),
            Float(toolPosePosition.z)
        )

        return SpatialTargetingMath.solveCalibration(
            baseWorldPoint: pendingBasePlacement.baseWorldPoint,
            gripperWorldPoint: gripperWorldPoint,
            toolPoseInRobotFrame: toolVector,
            surfaceNormal: pendingBasePlacement.surfaceNormal
        )?.robotToWorldTransform
    }

    private func isEligible(_ anchor: PlaneAnchor) -> Bool {
        guard anchor.alignment == .horizontal else { return false }
        guard anchor.surfaceClassification == .table || anchor.surfaceClassification == .floor else { return false }
        return anchor.geometry.extent.width > 0.25 && anchor.geometry.extent.height > 0.25
    }

    private func previewState(
        for targetWorldPoint: SIMD3<Float>,
        robotToWorldTransform: simd_float4x4
    ) -> SpatialSessionModel.SpatialPreviewState {
        let seedJointPositions = spatial.previewState?.previewJointPositions ?? state.robotState.jointPositions
        return SpatialTargetingMath.previewState(
            targetWorldPoint: targetWorldPoint,
            robotToWorldTransform: robotToWorldTransform,
            workspaceBounds: connection.workspaceBounds,
            seedJointPositions: seedJointPositions,
            maxIKErrorMeters: jointStreamingPolicy.maxIKErrorMeters
        )
    }

    private func pauseJointStreamingForInvalidTarget(reason: String) {
        guard !jointStreamingPausedForInvalidTarget else { return }
        jointStreamingPausedForInvalidTarget = true
        Task { await connection.stopJointPreviewStreaming() }
        commandFlow.appendTranscript(.system, reason)
    }

    private func invalidTargetMessage(
        for reason: SpatialSessionModel.SpatialTargetInvalidReason?,
        stage: SpatialSessionModel.SpatialOnboardingStage
    ) -> String {
        switch reason {
        case .noSurface:
            return "Move the stylus onto a recognized table or floor surface before confirming the robot base."
        case .outsideWorkspace:
            if stage == .placingGripper {
                return "The stylus point is outside the robot workspace. Bring it back into the green zone before finishing calibration."
            }
            return "The stylus point is outside the robot workspace. Bring it back into the green zone before starting or continuing control."
        case .ikError:
            if stage == .placingGripper {
                return "The stylus point is inside the workspace, but the arm cannot reach it cleanly yet. Move back to a green target before finishing calibration."
            }
            return "The stylus point is not currently safe for control. Move back to a green target before starting or continuing control."
        case .unavailable, .none:
            return "The stylus target could not be resolved."
        }
    }

    private func setMarkerColor(_ marker: ModelEntity?, isValid: Bool) {
        guard let marker else { return }
        let color = isValid
            ? UIColor(red: 0.18, green: 0.96, blue: 0.42, alpha: 0.98)
            : UIColor(red: 1.0, green: 0.27, blue: 0.27, alpha: 0.98)
        marker.model?.materials = [UnlitMaterial(color: color)]
    }

    // MARK: - Entity builders

    private func buildSurfaceOverlay() -> Entity {
        let material = SimpleMaterial(color: .init(red: 0.0, green: 0.6, blue: 1.0, alpha: 0.18), isMetallic: false)
        let plane = ModelEntity(mesh: .generatePlane(width: 0.18, depth: 0.18), materials: [material])
        plane.position = [0, 0.001, 0]
        let root = Entity()
        root.addChild(plane)
        return root
    }

    private func buildPlacementMarker() -> ModelEntity {
        let material = UnlitMaterial(color: UIColor(red: 1.0, green: 0.55, blue: 0.0, alpha: 1.0))
        return ModelEntity(mesh: .generateSphere(radius: 0.018), materials: [material])
    }

    private func buildTargetMarker() -> ModelEntity {
        let material = UnlitMaterial(color: UIColor(red: 0.0, green: 1.0, blue: 0.45, alpha: 0.98))
        return ModelEntity(mesh: .generateSphere(radius: 0.016), materials: [material])
    }

    private func buildWorkspaceVolume(bounds: WorkspaceBounds) -> Entity {
        let root = Entity()
        let material = UnlitMaterial(color: UIColor(red: 0.0, green: 0.78, blue: 1.0, alpha: 0.32))
        let cornerMaterial = UnlitMaterial(color: UIColor(red: 1.0, green: 0.45, blue: 0.08, alpha: 0.72))
        let corners = volumeCorners(bounds: bounds)
        let edges = [
            (0, 1), (0, 2), (0, 4),
            (1, 3), (1, 5),
            (2, 3), (2, 6),
            (3, 7),
            (4, 5), (4, 6),
            (5, 7),
            (6, 7),
        ]
        for edge in edges {
            root.addChild(buildLine(from: corners[edge.0], to: corners[edge.1], radius: 0.0012, material: material))
        }
        for corner in corners {
            let marker = ModelEntity(mesh: .generateSphere(radius: 0.0042), materials: [cornerMaterial])
            marker.position = corner
            root.addChild(marker)
        }
        return root
    }

    private func buildLine(from start: SIMD3<Float>, to end: SIMD3<Float>, radius: Float, material: UnlitMaterial) -> ModelEntity {
        let vector = end - start
        let length = simd_length(vector)
        let entity = ModelEntity(mesh: .generateCylinder(height: length, radius: radius), materials: [material])
        entity.position = (start + end) * 0.5
        if length > 0.0001 {
            entity.orientation = simd_quatf(from: SIMD3<Float>(0, 1, 0), to: simd_normalize(vector))
        }
        return entity
    }

    private func volumeCorners(bounds: WorkspaceBounds) -> [SIMD3<Float>] {
        let x0 = Float(bounds.xMin)
        let x1 = Float(bounds.xMax)
        let y0 = Float(bounds.yMin)
        let y1 = Float(bounds.yMax)
        let z0 = Float(bounds.zMin)
        let z1 = Float(bounds.zMax)
        return [
            SIMD3<Float>(x0, y0, z0),
            SIMD3<Float>(x1, y0, z0),
            SIMD3<Float>(x0, y1, z0),
            SIMD3<Float>(x1, y1, z0),
            SIMD3<Float>(x0, y0, z1),
            SIMD3<Float>(x1, y0, z1),
            SIMD3<Float>(x0, y1, z1),
            SIMD3<Float>(x1, y1, z1),
        ]
    }

    private func rotationAligningUp(to normal: SIMD3<Float>) -> simd_quatf {
        let normalized = simd_length_squared(normal) > 0.000001 ? simd_normalize(normal) : SIMD3<Float>(0, 1, 0)
        return simd_quatf(from: SIMD3<Float>(0, 1, 0), to: normalized)
    }

    // MARK: - Command dispatch

    private func sendGoal(_ position: SIMD3<Float>) async {
        await spatial.sendCartesianGoal(
            x: Double(position.x),
            y: Double(position.y),
            z: Double(position.z)
        )
        cancelConfirmation()
    }

    // MARK: - Anchor helpers

    private func createWorldAnchor(transform: simd_float4x4) async throws -> WorldAnchor {
        let anchor = WorldAnchor(originFromAnchorTransform: transform)
        try await worldTracking.addAnchor(anchor)
        return anchor
    }

    private func removeCalibrationWorldAnchor() async {
        guard let worldAnchorID = spatial.calibrationPayload?.worldAnchorID else { return }
        try? await worldTracking.removeAnchor(forID: worldAnchorID)
    }

    @MainActor
    private func upsertPlaneVisualization(for anchor: PlaneAnchor) async {
        guard let planeVisualizationRoot else { return }

        do {
            let mesh = try await MeshResource(from: anchor)
            let shape = try await ShapeResource.generateStaticMesh(
                positions: anchor.geometry.meshVertices.asSIMD3FloatArray(),
                faceIndices: anchor.geometry.meshFaces.asUInt16Array()
            )
            let material = UnlitMaterial(color: planeVisualizationColor(for: anchor))
            let visualization = planeVisualizations[anchor.id] ?? ModelEntity()
            visualization.model = ModelComponent(mesh: mesh, materials: [material])
            visualization.transform = Transform(matrix: anchor.originFromAnchorTransform)
            visualization.position += planeNormal(for: anchor) * 0.002
            let collisionGroup = anchor.alignment == .horizontal
                ? PlaneAnchor.horizontalCollisionGroup
                : PlaneAnchor.verticalCollisionGroup
            visualization.components.set(
                CollisionComponent(
                    shapes: [shape],
                    isStatic: true,
                    filter: CollisionFilter(group: collisionGroup, mask: .all)
                )
            )
            visualization.components.set(PlaneAnchorIDComponent(anchorID: anchor.id))
            if visualization.parent == nil {
                planeVisualizationRoot.addChild(visualization)
            }
            planeVisualizations[anchor.id] = visualization
        } catch {
            commandFlow.appendTranscript(.error, "Plane visualization failed: \(error.localizedDescription)")
        }
    }

    @MainActor
    private func removePlaneVisualization(for anchorID: UUID) async {
        planeVisualizations.removeValue(forKey: anchorID)?.removeFromParent()
    }

    private func planeVisualizationColor(for anchor: PlaneAnchor) -> UIColor {
        switch anchor.surfaceClassification {
        case .table:
            return UIColor(red: 0.0, green: 0.78, blue: 1.0, alpha: 0.10)
        case .floor:
            return UIColor(red: 0.18, green: 0.56, blue: 1.0, alpha: 0.06)
        default:
            return UIColor(red: 0.0, green: 0.78, blue: 1.0, alpha: 0.08)
        }
    }

    private func planeNormal(for anchor: PlaneAnchor) -> SIMD3<Float> {
        var normal = SIMD3<Float>(
            anchor.originFromAnchorTransform.columns.1.x,
            anchor.originFromAnchorTransform.columns.1.y,
            anchor.originFromAnchorTransform.columns.1.z
        )
        if simd_dot(normal, SIMD3<Float>(0, 1, 0)) < 0 {
            normal *= -1
        }
        return simd_normalize(normal)
    }
}

// MARK: - Helpers

private struct SurfacePlacement {
    let anchorID: UUID
    let worldPoint: SIMD3<Float>
    let surfaceNormal: SIMD3<Float>
    let distanceToTip: Float
    let score: Float
    let resolutionMethod: SpatialPlaneProjector.ResolutionMethod
    let comparisonDeltaMeters: Float?
}

private struct PlaneAnchorIDComponent: Component {
    let anchorID: UUID
}

private struct CartesianConfirmPanel: View {
    let position: SIMD3<Float>?
    let onConfirm: (SIMD3<Float>) -> Void
    let onCancel: () -> Void

    var body: some View {
        if let pos = position {
            VStack(spacing: 16) {
                Image(systemName: "checkmark.circle")
                    .font(.system(size: 42))
                    .foregroundStyle(.green)

                Text("Confirm the Cartesian move")
                    .font(.headline.weight(.bold))
                    .multilineTextAlignment(.center)

                Text("The green marker shows the resolved gripper target. Review the base-frame target below, then send or cancel.")
                    .font(.subheadline.weight(.medium))
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                    .frame(maxWidth: 300)

                Text(String(format: "(%.3f, %.3f, %.3f) m", pos.x, pos.y, pos.z))
                    .font(.callout.monospaced().weight(.semibold))
                    .foregroundStyle(.primary)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 8)
                    .background(.white.opacity(0.08), in: Capsule())

                HStack(spacing: 16) {
                    Button("Cancel") {
                        onCancel()
                    }
                    .frame(minWidth: 120)
                    .buttonStyle(.bordered)
                    .tint(.secondary)

                    Button("Send") {
                        onConfirm(pos)
                    }
                    .frame(minWidth: 120)
                    .buttonStyle(.borderedProminent)
                    .tint(.green)
                }
                .padding(.top, 4)
            }
            .frame(width: 360)
            .padding(24)
            .glassBackgroundEffect()
        }
    }
}
