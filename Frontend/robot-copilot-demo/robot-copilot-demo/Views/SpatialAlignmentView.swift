import SwiftUI
import RealityKit
import ARKit
import RosClawBridge

struct SpatialAlignmentView: View {
    @Environment(AppModel.self) private var appModel
    @Environment(SpatialStylusModel.self) private var stylusModel

    private let arkitSession = ARKitSession()
    private let worldTracking = WorldTrackingProvider()
    private let planeDetection = PlaneDetectionProvider(alignments: [.horizontal])

    @State private var sceneReferenceEntity: Entity?
    @State private var workspaceAnchorEntity: Entity?
    @State private var placedOnPlane = false
    @State private var targetMarkerEntity: Entity?
    @State private var robotBaseWorldPosition: SIMD3<Float>?
    @State private var isCalibrated = false
    @State private var pendingGoalPosition: SIMD3<Float>?
    @State private var confirmLabelEntity: Entity?
    @State private var stylusTriggerPressed = false
    @State private var stylusPreviewWorldPosition: SIMD3<Float>?
    @State private var sceneUpdateSubscription: EventSubscription?

    var body: some View {
        RealityView { content, attachments in
            let root = Entity()
            content.add(root)
            sceneReferenceEntity = root

            let workspace = buildWorkspaceOverlay()
            workspace.isEnabled = false
            root.addChild(workspace)
            workspaceAnchorEntity = workspace

            let marker = buildTargetMarker()
            marker.isEnabled = false
            root.addChild(marker)
            targetMarkerEntity = marker

            if let confirmPanel = attachments.entity(for: "confirm-goal") {
                confirmPanel.isEnabled = false
                root.addChild(confirmPanel)
                confirmLabelEntity = confirmPanel
            }

            sceneUpdateSubscription = content.subscribe(to: SceneEvents.Update.self) { _ in
                Task { @MainActor in
                    guard let root = sceneReferenceEntity else { return }
                    processStylusUpdates(relativeTo: root)
                }
            }
        } update: { _, _ in
        } attachments: {
            Attachment(id: "confirm-goal") {
                CartesianConfirmView(
                    position: pendingGoalPosition,
                    onConfirm: { pos in
                        Task { await sendGoal(pos) }
                    },
                    onCancel: {
                        cancelTapConfirmation()
                    }
                )
                .environment(appModel)
            }
        }
        .gesture(
            SpatialTapGesture()
                .targetedToAnyEntity()
                .onEnded { value in
                    handleSpatialTap(.init(value.location3D))
                }
        )
        .task {
            await runARKit()
        }
        .task {
            await processPlaneUpdates()
        }
        .onDisappear {
            sceneUpdateSubscription?.cancel()
            sceneUpdateSubscription = nil
        }
    }

    private func handleSpatialTap(_ worldPosition: SIMD3<Float>) {
        if !isCalibrated {
            robotBaseWorldPosition = worldPosition
            isCalibrated = true
            appModel.registerSpatialCalibration(worldPosition: worldPosition)
            appModel.appendTranscript(.system, "Robot base calibrated at spatial position")

            if let workspace = workspaceAnchorEntity {
                workspace.position = worldPosition
                workspace.isEnabled = true
            }
            return
        }

        guard let goalPos = robotGoalPosition(forWorldPosition: worldPosition) else { return }
        updateTargetMarker(worldPosition)
        pendingGoalPosition = goalPos
        appModel.setPendingSpatialGoal(goalPos)
        confirmLabelEntity?.position = worldPosition + SIMD3<Float>(0, 0.06, 0)
        confirmLabelEntity?.isEnabled = true
    }

    private func processStylusUpdates(relativeTo root: Entity) {
        let events = stylusModel.getLatestButtonEvents()

        for event in events {
            if event.source == .secondary, event.isPressed {
                cancelStylusPreview()
                continue
            }

            if event.isPressed {
                stylusTriggerPressed = true
            } else if stylusTriggerPressed {
                stylusTriggerPressed = false
                commitStylusPreview(relativeTo: root)
            }
        }

        guard stylusTriggerPressed else { return }
        guard isCalibrated else { return }
        guard let previewPosition = stylusModel.predictedTipPosition(relativeTo: root) else { return }

        cancelTapConfirmation()
        stylusPreviewWorldPosition = previewPosition
        updateTargetMarker(previewPosition)
    }

    private func commitStylusPreview(relativeTo root: Entity) {
        guard isCalibrated else {
            appModel.appendTranscript(.error, "Calibrate the robot base before using the stylus.")
            cancelStylusPreview()
            return
        }

        guard let worldPosition = stylusModel.latestTipPosition(relativeTo: root) ?? stylusPreviewWorldPosition,
              let goalPosition = robotGoalPosition(forWorldPosition: worldPosition)
        else {
            appModel.appendTranscript(.error, "Stylus target could not be resolved.")
            cancelStylusPreview()
            return
        }

        let command = RobotCommand.moveToCartesian(
            x: Double(goalPosition.x),
            y: Double(goalPosition.y),
            z: Double(goalPosition.z)
        )
        let safetyResult = appModel.safetyGate.validate(command, robotState: appModel.robotState)
        guard safetyResult.isAllowed else {
            appModel.appendTranscript(
                .error,
                safetyResult.blockReason ?? "Stylus target is outside the safe workspace."
            )
            cancelStylusPreview()
            return
        }

        updateTargetMarker(worldPosition)
        stylusPreviewWorldPosition = nil
        Task {
            await appModel.sendCartesianGoal(
                x: Double(goalPosition.x),
                y: Double(goalPosition.y),
                z: Double(goalPosition.z)
            )
            try? await Task.sleep(nanoseconds: 2_000_000_000)
            await MainActor.run {
                targetMarkerEntity?.isEnabled = false
            }
        }
    }

    private func cancelStylusPreview() {
        stylusTriggerPressed = false
        stylusPreviewWorldPosition = nil
        targetMarkerEntity?.isEnabled = false
    }

    private func cancelTapConfirmation() {
        pendingGoalPosition = nil
        appModel.clearPendingSpatialGoal()
        confirmLabelEntity?.isEnabled = false
        if !stylusTriggerPressed {
            targetMarkerEntity?.isEnabled = false
        }
    }

    private func robotGoalPosition(forWorldPosition worldPosition: SIMD3<Float>) -> SIMD3<Float>? {
        guard let basePosition = robotBaseWorldPosition else {
            return nil
        }
        return SpatialTargetingMath.robotPosition(
            fromWorldPosition: worldPosition,
            relativeToRobotBase: basePosition
        )
    }

    private func updateTargetMarker(_ worldPosition: SIMD3<Float>) {
        targetMarkerEntity?.position = worldPosition
        targetMarkerEntity?.isEnabled = true
    }

    private func sendGoal(_ position: SIMD3<Float>) async {
        await appModel.sendCartesianGoal(
            x: Double(position.x),
            y: Double(position.y),
            z: Double(position.z)
        )
        cancelTapConfirmation()
        try? await Task.sleep(nanoseconds: 2_000_000_000)
        targetMarkerEntity?.isEnabled = false
    }

    private func runARKit() async {
        guard WorldTrackingProvider.isSupported && PlaneDetectionProvider.isSupported else { return }
        do {
            try await arkitSession.run([worldTracking, planeDetection])
        } catch {
            await MainActor.run {
                appModel.appendTranscript(.error, "ARKit session failed: \(error.localizedDescription)")
            }
        }
    }

    private func processPlaneUpdates() async {
        for await update in planeDetection.anchorUpdates {
            let anchor = update.anchor
            guard !placedOnPlane,
                  anchor.surfaceClassification == .table || anchor.surfaceClassification == .floor,
                  anchor.geometry.extent.width > 0.3,
                  anchor.geometry.extent.height > 0.3 else { continue }

            await MainActor.run {
                if !isCalibrated, let workspace = workspaceAnchorEntity {
                    let transform = Transform(matrix: anchor.originFromAnchorTransform)
                    workspace.transform = transform
                    workspace.isEnabled = true
                }
                placedOnPlane = true
                appModel.appendTranscript(
                    .system,
                    "Table surface detected. Tap the robot base to calibrate, then tap a target or hold the stylus trigger to preview and release to move."
                )
            }
        }
    }

    private func buildWorkspaceOverlay() -> Entity {
        let root = Entity()
        root.name = "workspace_overlay"

        var planeMaterial = SimpleMaterial()
        planeMaterial.color = .init(tint: .init(red: 0, green: 0.5, blue: 1, alpha: 0.15))
        let plane = ModelEntity(
            mesh: .generatePlane(width: 0.5, depth: 0.5),
            materials: [planeMaterial]
        )
        plane.position = [0, 0.001, 0]
        root.addChild(plane)

        let markerMaterial = SimpleMaterial(color: .init(red: 0, green: 0.8, blue: 1, alpha: 0.8), isMetallic: false)
        for x: Float in [-0.25, 0.25] {
            for z: Float in [-0.25, 0.25] {
                let marker = ModelEntity(mesh: .generateSphere(radius: 0.01), materials: [markerMaterial])
                marker.position = [x, 0.01, z]
                root.addChild(marker)
            }
        }

        let baseMaterial = SimpleMaterial(color: .init(red: 1, green: 0.3, blue: 0, alpha: 0.9), isMetallic: false)
        let baseMarker = ModelEntity(mesh: .generateSphere(radius: 0.015), materials: [baseMaterial])
        baseMarker.name = "base_indicator"
        baseMarker.position = [0, 0.015, 0]
        root.addChild(baseMarker)

        root.components.set(InputTargetComponent())
        root.generateCollisionShapes(recursive: true)
        return root
    }

    private func buildTargetMarker() -> Entity {
        let material = SimpleMaterial(color: .init(red: 0, green: 1, blue: 0.4, alpha: 0.9), isMetallic: false)
        let marker = ModelEntity(mesh: .generateSphere(radius: 0.018), materials: [material])
        marker.name = "cartesian_target"
        return marker
    }
}

private struct CartesianConfirmView: View {
    let position: SIMD3<Float>?
    let onConfirm: (SIMD3<Float>) -> Void
    let onCancel: () -> Void

    @Environment(AppModel.self) private var appModel

    var body: some View {
        if let pos = position {
            VStack(spacing: 8) {
                Text(String(format: "Move to (%.3f, %.3f, %.3f)?", pos.x, pos.y, pos.z))
                    .font(.caption)
                    .foregroundStyle(.primary)

                HStack(spacing: 12) {
                    Button("Cancel") { onCancel() }
                        .buttonStyle(.bordered)
                        .tint(.secondary)
                        .controlSize(.small)

                    Button("Send") { onConfirm(pos) }
                        .buttonStyle(.borderedProminent)
                        .tint(.green)
                        .controlSize(.small)
                }
            }
            .padding(10)
            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 10))
        }
    }
}
