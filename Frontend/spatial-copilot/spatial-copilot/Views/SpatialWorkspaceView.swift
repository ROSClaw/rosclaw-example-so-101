import ARKit
import QuartzCore
import RealityKit
import RosClawBridge
import Spatial
import SwiftUI

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

    @State private var sceneRoot: Entity?
    @State private var robotModel: RobotModelEntity?
    @State private var workspaceVolume: Entity?
    @State private var lastWorkspaceBounds: WorkspaceBounds?
    @State private var surfaceOverlay: Entity?
    @State private var placementMarker: ModelEntity?
    @State private var targetMarker: ModelEntity?
    @State private var statusBadge: Entity?
    @State private var confirmPanel: Entity?
    @State private var guidePanel: Entity?
    @State private var accessoryTrackingProvider: AccessoryTrackingProvider?
    @State private var planeAnchors: [UUID: PlaneAnchor] = [:]
    @State private var currentSurfacePlacement: SurfacePlacement?
    @State private var currentTipWorldPoint: SIMD3<Float>?
    @State private var surfaceHintAnnounced = false
    @State private var sceneUpdateSubscription: EventSubscription?

    var body: some View {
        RealityView { content, attachments in
            let root = Entity()
            content.add(root)
            sceneRoot = root

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
                root.addChild(panel)
                confirmPanel = panel
            }
            if let guide = attachments.entity(for: "calibration-guide") {
                guide.isEnabled = true
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
            await maintainRobotPoseRefreshLoop()
        }
        .onDisappear {
            sceneUpdateSubscription?.cancel()
            sceneUpdateSubscription = nil
            planeAnchors.removeAll()
            arkitSession.stop()
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
            currentTipWorldPoint =
                stylusTipPosition(relativeTo: root, predicted: true) ??
                stylusTipPosition(relativeTo: root, predicted: false)
        }

        renderCurrentStage(relativeTo: root)
        handleButtonEvents()
        renderCurrentStage(relativeTo: root)
        updateStatusBadgePosition()
        updateGuidePanelPosition()
    }

    private func renderCurrentStage(relativeTo root: Entity) {
        switch spatial.onboardingStage {
        case .needsStylus, .needsRobotState:
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
            spatial.setPreviewState(nil)
            setAppPhase(.idle)
            targetMarker?.isEnabled = false
            confirmPanel?.isEnabled = false
            renderBasePlacement()
        case .placingGripper:
            spatial.setPreviewState(nil)
            setAppPhase(.idle)
            targetMarker?.isEnabled = false
            confirmPanel?.isEnabled = false
            renderGripperPlacement()
        case .previewing:
            setAppPhase(.previewing)
            confirmPanel?.isEnabled = false
            renderRobotPreview()
        case .confirming:
            setAppPhase(.awaitingConfirmation)
            renderConfirmation()
        }

        syncRobotPresentation()
    }

    private func renderBasePlacement() {
        guard let tip = currentTipWorldPoint,
              let surfacePlacement = resolvedSurfacePlacement(for: tip) else {
            currentSurfacePlacement = nil
            surfaceOverlay?.isEnabled = false
            placementMarker?.isEnabled = false
            return
        }

        currentSurfacePlacement = surfacePlacement
        placementMarker?.model?.materials = [SimpleMaterial(color: .orange, isMetallic: false)]
        placementMarker?.position = surfacePlacement.worldPoint
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
        guard let tip = currentTipWorldPoint else {
            placementMarker?.isEnabled = false
            return
        }
        placementMarker?.model?.materials = [SimpleMaterial(color: .cyan, isMetallic: false)]
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

        let requestedRobotPoint = SpatialTargetingMath.robotPosition(
            fromWorldPosition: tip,
            using: calibration.worldToRobotTransform
        )
        let clampedRobotPoint = SpatialTargetingMath.clamp(requestedRobotPoint, to: connection.workspaceBounds)
        let resolvedWorldPoint = SpatialTargetingMath.worldPosition(
            fromRobotPosition: clampedRobotPoint,
            using: calibration.robotToWorldTransform
        )
        let ikResult = SO101PreviewIK().solve(
            targetInBaseFrame: SIMD3<Double>(
                Double(clampedRobotPoint.x),
                Double(clampedRobotPoint.y),
                Double(clampedRobotPoint.z)
            ),
            seededFromJointPositions: state.robotState.jointPositions,
            preserveWristRoll: true
        )
        let endEffectorWorldPoint = SpatialTargetingMath.worldPosition(
            fromRobotPosition: SIMD3<Float>(
                Float(ikResult.endEffectorPosition.x),
                Float(ikResult.endEffectorPosition.y),
                Float(ikResult.endEffectorPosition.z)
            ),
            using: calibration.robotToWorldTransform
        )

        spatial.setPreviewState(
            .init(
                targetWorldPoint: resolvedWorldPoint,
                requestedRobotPoint: requestedRobotPoint,
                clampedRobotPoint: clampedRobotPoint,
                insideWorkspace: SpatialTargetingMath.isInside(requestedRobotPoint, bounds: connection.workspaceBounds),
                previewJointPositions: ikResult.jointAngles.asJointPositionMap,
                previewEndEffectorWorldPoint: endEffectorWorldPoint,
                positionErrorMeters: ikResult.positionErrorMeters
            )
        )

        targetMarker?.position = resolvedWorldPoint
        targetMarker?.isEnabled = true
    }

    private func renderConfirmation() {
        surfaceOverlay?.isEnabled = false
        placementMarker?.isEnabled = false
        confirmPanel?.isEnabled = spatial.pendingSpatialRobotGoal != nil
        if let worldPoint = spatial.pendingGoalWorldPoint {
            targetMarker?.position = worldPoint
            targetMarker?.isEnabled = true
            confirmPanel?.position = worldPoint + SIMD3<Float>(0, 0.08, 0)
        }
    }

    // MARK: - Input handling

    private func handleButtonEvents() {
        for event in stylusModel.getLatestButtonEvents() where event.isPressed {
            switch event.source {
            case .primary:
                handlePrimaryPress()
            case .secondary:
                handleSecondaryPress()
            case .tip, .none:
                break
            }
        }
    }

    private func handlePrimaryPress() {
        switch spatial.onboardingStage {
        case .needsStylus:
            commandFlow.appendTranscript(.error, "Connect the spatial stylus before starting immersive calibration.")
        case .needsRobotState:
            commandFlow.appendTranscript(.error, "Live robot gripper pose is still unavailable.")
        case .placingBase:
            guard let surfacePlacement = currentSurfacePlacement else {
                commandFlow.appendTranscript(.error, "Move the stylus onto a recognized surface before confirming the robot base.")
                return
            }
            spatial.registerBasePlacement(
                surfaceAnchorID: surfacePlacement.anchorID,
                surfaceNormal: surfacePlacement.surfaceNormal,
                worldPoint: surfacePlacement.worldPoint
            )
            commandFlow.appendTranscript(.system, "Robot base placed. Now point the stylus at the gripper location and press the main button again.")
        case .placingGripper:
            guard let tip = currentTipWorldPoint,
                  let toolPose = state.robotState.toolPose,
                  spatial.completeCalibration(gripperWorldPoint: tip, toolPose: toolPose) else {
                commandFlow.appendTranscript(.error, "The virtual robot could not be calibrated from the sampled base and gripper points.")
                return
            }
            commandFlow.appendTranscript(.system, "Calibration complete. Hover the stylus inside the workspace to preview the robot and press the main button to review a Cartesian move.")
        case .previewing:
            guard let preview = spatial.previewState else {
                commandFlow.appendTranscript(.error, "The stylus target could not be resolved.")
                return
            }
            spatial.setPendingSpatialGoal(preview.clampedRobotPoint, worldPoint: preview.targetWorldPoint)
            if !preview.insideWorkspace {
                commandFlow.appendTranscript(.system, "Target clamped to the nearest safe workspace point before confirmation.")
            }
        case .confirming:
            break
        }
    }

    private func handleSecondaryPress() {
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
            if isEligible(anchor) {
                if !surfaceHintAnnounced {
                    surfaceHintAnnounced = true
                    commandFlow.appendTranscript(.system, "Surface recognized. Use the stylus to place the base on the surface, then sample the gripper in free space.")
                }
            }

            switch update.event {
            case .added, .updated:
                if isEligible(anchor) {
                    planeAnchors[anchor.id] = anchor
                } else {
                    planeAnchors.removeValue(forKey: anchor.id)
                }
            case .removed:
                planeAnchors.removeValue(forKey: anchor.id)
            }
        }
    }

    private func maintainRobotPoseRefreshLoop() async {
        while !Task.isCancelled {
            if spatial.onboardingStage == .needsRobotState,
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

    // MARK: - Presentation helpers

    private func syncRobotPresentation() {
        if let calibration = spatial.calibrationPayload {
            robotModel?.setCalibrationTransform(calibration.robotToWorldTransform)
            robotModel?.rootEntity.isEnabled = true
            workspaceVolume?.transform = Transform(matrix: calibration.robotToWorldTransform)
            workspaceVolume?.isEnabled = true
        } else {
            robotModel?.rootEntity.isEnabled = false
            workspaceVolume?.isEnabled = false
        }

        if let preview = spatial.previewState {
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
        volume.isEnabled = spatial.calibrationPayload != nil
        root.addChild(volume)
        workspaceVolume = volume
        lastWorkspaceBounds = connection.workspaceBounds
    }

    private func updateStatusBadgePosition() {
        guard let badge = statusBadge else { return }
        if let previewPoint = spatial.previewState?.previewEndEffectorWorldPoint {
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
        case .confirming:
            guide.position = (spatial.pendingGoalWorldPoint ?? spatial.calibrationPayload?.baseWorldPoint ?? SIMD3<Float>(0, 1.1, -0.6)) + SIMD3<Float>(0, 0.12, 0)
            guide.isEnabled = true
        case .previewing:
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
        var best: SurfacePlacement?

        for anchor in planeAnchors.values where isEligible(anchor) {
            let extentTransform = anchor.originFromAnchorTransform * anchor.geometry.extent.anchorFromExtentTransform
            let localPoint = SpatialTargetingMath.transform(point: point, with: extentTransform.inverse)
            let halfWidth = anchor.geometry.extent.width * 0.5
            let halfDepth = anchor.geometry.extent.height * 0.5
            let clampedLocal = SIMD3<Float>(
                x: min(max(localPoint.x, -halfWidth), halfWidth),
                y: 0,
                z: min(max(localPoint.z, -halfDepth), halfDepth)
            )
            let worldPoint = SpatialTargetingMath.transform(point: clampedLocal, with: extentTransform)
            var normal = SIMD3<Float>(
                extentTransform.columns.1.x,
                extentTransform.columns.1.y,
                extentTransform.columns.1.z
            )
            if simd_dot(normal, SIMD3<Float>(0, 1, 0)) < 0 {
                normal *= -1
            }
            let placement = SurfacePlacement(
                anchorID: anchor.id,
                worldPoint: worldPoint,
                surfaceNormal: simd_normalize(normal),
                distanceToTip: simd_length(worldPoint - point)
            )
            if best == nil || placement.distanceToTip < best!.distanceToTip {
                best = placement
            }
        }

        return best
    }

    private func isEligible(_ anchor: PlaneAnchor) -> Bool {
        guard anchor.alignment == .horizontal else { return false }
        guard anchor.surfaceClassification == .table || anchor.surfaceClassification == .floor else { return false }
        return anchor.geometry.extent.width > 0.25 && anchor.geometry.extent.height > 0.25
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
        let material = SimpleMaterial(color: .orange, isMetallic: false)
        return ModelEntity(mesh: .generateSphere(radius: 0.014), materials: [material])
    }

    private func buildTargetMarker() -> ModelEntity {
        let material = SimpleMaterial(color: .init(red: 0.0, green: 1.0, blue: 0.45, alpha: 0.95), isMetallic: false)
        return ModelEntity(mesh: .generateSphere(radius: 0.016), materials: [material])
    }

    private func buildWorkspaceVolume(bounds: WorkspaceBounds) -> Entity {
        let root = Entity()
        let material = SimpleMaterial(color: .init(red: 0.0, green: 0.8, blue: 1.0, alpha: 0.85), isMetallic: false)
        let cornerMaterial = SimpleMaterial(color: .init(red: 1.0, green: 0.4, blue: 0.1, alpha: 0.95), isMetallic: false)
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
            root.addChild(buildLine(from: corners[edge.0], to: corners[edge.1], radius: 0.0018, material: material))
        }
        for corner in corners {
            let marker = ModelEntity(mesh: .generateSphere(radius: 0.0055), materials: [cornerMaterial])
            marker.position = corner
            root.addChild(marker)
        }
        return root
    }

    private func buildLine(from start: SIMD3<Float>, to end: SIMD3<Float>, radius: Float, material: SimpleMaterial) -> ModelEntity {
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
}

// MARK: - Helpers

private struct SurfacePlacement {
    let anchorID: UUID
    let worldPoint: SIMD3<Float>
    let surfaceNormal: SIMD3<Float>
    let distanceToTip: Float
}

private struct CartesianConfirmPanel: View {
    let position: SIMD3<Float>?
    let onConfirm: (SIMD3<Float>) -> Void
    let onCancel: () -> Void

    var body: some View {
        if let pos = position {
            VStack(spacing: 12) {
                Text(String(format: "Move gripper to (%.3f, %.3f, %.3f) m?", pos.x, pos.y, pos.z))
                    .font(.callout.weight(.medium))
                    .multilineTextAlignment(.center)
                HStack(spacing: 16) {
                    Button("Cancel") {
                        onCancel()
                    }
                    .buttonStyle(.bordered)
                    .tint(.secondary)

                    Button("Send") {
                        onConfirm(pos)
                    }
                    .buttonStyle(.borderedProminent)
                    .tint(.green)
                }
            }
            .padding(16)
            .glassBackgroundEffect()
        }
    }
}
