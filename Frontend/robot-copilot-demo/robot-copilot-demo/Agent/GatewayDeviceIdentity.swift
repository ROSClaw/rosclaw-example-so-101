import CryptoKit
import Foundation
#if canImport(UIKit)
import UIKit
#endif
import OpenClawProtocol

struct GatewayDeviceIdentity: Codable, Sendable {
    let deviceId: String
    let publicKey: String
    let privateKey: String
    let createdAtMs: Int

    nonisolated init(deviceId: String, publicKey: String, privateKey: String, createdAtMs: Int) {
        self.deviceId = deviceId
        self.publicKey = publicKey
        self.privateKey = privateKey
        self.createdAtMs = createdAtMs
    }

    private enum CodingKeys: String, CodingKey {
        case deviceId
        case publicKey
        case privateKey
        case createdAtMs
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.deviceId = try container.decode(String.self, forKey: .deviceId)
        self.publicKey = try container.decode(String.self, forKey: .publicKey)
        self.privateKey = try container.decode(String.self, forKey: .privateKey)
        self.createdAtMs = try container.decode(Int.self, forKey: .createdAtMs)
    }

    nonisolated func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(self.deviceId, forKey: .deviceId)
        try container.encode(self.publicKey, forKey: .publicKey)
        try container.encode(self.privateKey, forKey: .privateKey)
        try container.encode(self.createdAtMs, forKey: .createdAtMs)
    }
}

enum GatewayDeviceIdentityPaths {
    nonisolated private static let stateDirEnv = ["OPENCLAW_STATE_DIR"]

    nonisolated static func stateDirURL() -> URL {
        for key in self.stateDirEnv {
            if let raw = getenv(key) {
                let value = String(cString: raw).trimmingCharacters(in: .whitespacesAndNewlines)
                if !value.isEmpty {
                    return URL(fileURLWithPath: value, isDirectory: true)
                }
            }
        }

        if let appSupport = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask).first {
            return appSupport.appendingPathComponent("OpenClaw", isDirectory: true)
        }

        return FileManager.default.temporaryDirectory.appendingPathComponent("openclaw", isDirectory: true)
    }
}

enum GatewayDeviceIdentityStore {
    nonisolated private static let fileName = "device.json"

    nonisolated static func loadOrCreate() -> GatewayDeviceIdentity {
        let url = self.fileURL()
        if let data = try? Data(contentsOf: url),
           let decoded = try? JSONDecoder().decode(GatewayDeviceIdentity.self, from: data),
           !decoded.deviceId.isEmpty,
           !decoded.publicKey.isEmpty,
           !decoded.privateKey.isEmpty
        {
            return decoded
        }

        let identity = self.generate()
        self.save(identity)
        return identity
    }

    nonisolated static func signPayload(_ payload: String, identity: GatewayDeviceIdentity) -> String? {
        guard let privateKeyData = Data(base64Encoded: identity.privateKey) else {
            return nil
        }
        do {
            let privateKey = try Curve25519.Signing.PrivateKey(rawRepresentation: privateKeyData)
            let signature = try privateKey.signature(for: Data(payload.utf8))
            return self.base64URLEncode(signature)
        } catch {
            return nil
        }
    }

    nonisolated static func publicKeyBase64URL(_ identity: GatewayDeviceIdentity) -> String? {
        guard let data = Data(base64Encoded: identity.publicKey) else {
            return nil
        }
        return self.base64URLEncode(data)
    }

    nonisolated private static func generate() -> GatewayDeviceIdentity {
        let privateKey = Curve25519.Signing.PrivateKey()
        let publicKey = privateKey.publicKey
        let publicKeyData = publicKey.rawRepresentation
        let privateKeyData = privateKey.rawRepresentation
        let deviceId = SHA256.hash(data: publicKeyData).map { String(format: "%02x", $0) }.joined()
        return GatewayDeviceIdentity(
            deviceId: deviceId,
            publicKey: publicKeyData.base64EncodedString(),
            privateKey: privateKeyData.base64EncodedString(),
            createdAtMs: Int(Date().timeIntervalSince1970 * 1000)
        )
    }

    nonisolated private static func save(_ identity: GatewayDeviceIdentity) {
        let url = self.fileURL()
        do {
            try FileManager.default.createDirectory(
                at: url.deletingLastPathComponent(),
                withIntermediateDirectories: true
            )
            let data = try JSONEncoder().encode(identity)
            try data.write(to: url, options: [.atomic])
        } catch {
            return
        }
    }

    nonisolated private static func fileURL() -> URL {
        GatewayDeviceIdentityPaths.stateDirURL()
            .appendingPathComponent("identity", isDirectory: true)
            .appendingPathComponent(fileName, isDirectory: false)
    }

    nonisolated private static func base64URLEncode(_ data: Data) -> String {
        data.base64EncodedString()
            .replacingOccurrences(of: "+", with: "-")
            .replacingOccurrences(of: "/", with: "_")
            .replacingOccurrences(of: "=", with: "")
    }
}

struct GatewayDeviceAuthEntry: Codable, Sendable {
    let token: String
    let role: String
    let scopes: [String]
    let updatedAtMs: Int

    nonisolated init(token: String, role: String, scopes: [String], updatedAtMs: Int) {
        self.token = token
        self.role = role
        self.scopes = scopes
        self.updatedAtMs = updatedAtMs
    }

    private enum CodingKeys: String, CodingKey {
        case token
        case role
        case scopes
        case updatedAtMs
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.token = try container.decode(String.self, forKey: .token)
        self.role = try container.decode(String.self, forKey: .role)
        self.scopes = try container.decode([String].self, forKey: .scopes)
        self.updatedAtMs = try container.decode(Int.self, forKey: .updatedAtMs)
    }

    nonisolated func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(self.token, forKey: .token)
        try container.encode(self.role, forKey: .role)
        try container.encode(self.scopes, forKey: .scopes)
        try container.encode(self.updatedAtMs, forKey: .updatedAtMs)
    }
}

enum GatewayDeviceAuthStore {
    nonisolated private static let fileName = "device-auth.json"

    nonisolated static func loadToken(deviceId: String, role: String) -> GatewayDeviceAuthEntry? {
        guard let store = readStore(), store.deviceId == deviceId else {
            return nil
        }
        return store.tokens[normalizeRole(role)]
    }

    nonisolated static func storeToken(
        deviceId: String,
        role: String,
        token: String,
        scopes: [String] = []
    ) {
        let normalizedRole = normalizeRole(role)
        var store = readStore()
        if store?.deviceId != deviceId {
            store = GatewayDeviceAuthStoreFile(version: 1, deviceId: deviceId, tokens: [:])
        }
        let entry = GatewayDeviceAuthEntry(
            token: token,
            role: normalizedRole,
            scopes: normalizeScopes(scopes),
            updatedAtMs: Int(Date().timeIntervalSince1970 * 1000)
        )
        store?.tokens[normalizedRole] = entry
        if let store {
            writeStore(store)
        }
    }

    nonisolated static func clearToken(deviceId: String, role: String) {
        guard var store = readStore(), store.deviceId == deviceId else {
            return
        }
        let normalizedRole = normalizeRole(role)
        guard store.tokens[normalizedRole] != nil else {
            return
        }
        store.tokens.removeValue(forKey: normalizedRole)
        writeStore(store)
    }

    nonisolated private static func normalizeRole(_ role: String) -> String {
        role.trimmingCharacters(in: .whitespacesAndNewlines)
    }

    nonisolated private static func normalizeScopes(_ scopes: [String]) -> [String] {
        let trimmed = scopes
            .map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
            .filter { !$0.isEmpty }
        return Array(Set(trimmed)).sorted()
    }

    nonisolated private static func fileURL() -> URL {
        GatewayDeviceIdentityPaths.stateDirURL()
            .appendingPathComponent("identity", isDirectory: true)
            .appendingPathComponent(fileName, isDirectory: false)
    }

    nonisolated private static func readStore() -> GatewayDeviceAuthStoreFile? {
        let url = fileURL()
        guard let data = try? Data(contentsOf: url) else {
            return nil
        }
        guard let decoded = try? JSONDecoder().decode(GatewayDeviceAuthStoreFile.self, from: data),
              decoded.version == 1
        else {
            return nil
        }
        return decoded
    }

    nonisolated private static func writeStore(_ store: GatewayDeviceAuthStoreFile) {
        let url = fileURL()
        do {
            try FileManager.default.createDirectory(
                at: url.deletingLastPathComponent(),
                withIntermediateDirectories: true
            )
            let data = try JSONEncoder().encode(store)
            try data.write(to: url, options: [.atomic])
            try? FileManager.default.setAttributes([.posixPermissions: 0o600], ofItemAtPath: url.path)
        } catch {
            return
        }
    }
}

private struct GatewayDeviceAuthStoreFile: Codable {
    let version: Int
    let deviceId: String
    var tokens: [String: GatewayDeviceAuthEntry]

    nonisolated init(version: Int, deviceId: String, tokens: [String: GatewayDeviceAuthEntry]) {
        self.version = version
        self.deviceId = deviceId
        self.tokens = tokens
    }

    private enum CodingKeys: String, CodingKey {
        case version
        case deviceId
        case tokens
    }

    nonisolated init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        self.version = try container.decode(Int.self, forKey: .version)
        self.deviceId = try container.decode(String.self, forKey: .deviceId)
        self.tokens = try container.decode([String: GatewayDeviceAuthEntry].self, forKey: .tokens)
    }

    nonisolated func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(self.version, forKey: .version)
        try container.encode(self.deviceId, forKey: .deviceId)
        try container.encode(self.tokens, forKey: .tokens)
    }
}

enum GatewayInstanceIdentity {
    nonisolated private static let suiteName = "ai.openclaw.shared"
    nonisolated private static let instanceIdKey = "instanceId"

    nonisolated private static var defaults: UserDefaults {
        UserDefaults(suiteName: suiteName) ?? .standard
    }

    #if canImport(UIKit)
    private nonisolated static func readMainActor<T: Sendable>(_ body: @MainActor () -> T) -> T {
        if Thread.isMainThread {
            return MainActor.assumeIsolated { body() }
        }
        return DispatchQueue.main.sync {
            MainActor.assumeIsolated { body() }
        }
    }
    #endif

    nonisolated static let instanceId: String = {
        let defaults = GatewayInstanceIdentity.defaults
        if let existing = defaults.string(forKey: instanceIdKey)?
            .trimmingCharacters(in: .whitespacesAndNewlines),
           !existing.isEmpty
        {
            return existing
        }
        let instanceId = UUID().uuidString.lowercased()
        defaults.set(instanceId, forKey: instanceIdKey)
        return instanceId
    }()

    nonisolated static let displayName: String = {
        #if canImport(UIKit)
        let name = Self.readMainActor {
            UIDevice.current.name.trimmingCharacters(in: .whitespacesAndNewlines)
        }
        return name.isEmpty ? "VisionOS OpenClaw" : name
        #else
        return "VisionOS OpenClaw"
        #endif
    }()

    nonisolated static let modelIdentifier: String? = {
        #if canImport(UIKit)
        var systemInfo = utsname()
        uname(&systemInfo)
        let machine = withUnsafeBytes(of: &systemInfo.machine) { ptr in
            String(bytes: ptr.prefix { $0 != 0 }, encoding: .utf8)
        }
        let trimmed = machine?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        return trimmed.isEmpty ? nil : trimmed
        #else
        return nil
        #endif
    }()

    nonisolated static let deviceFamily: String = "Vision"

    nonisolated static let platformString: String = {
        let version = ProcessInfo.processInfo.operatingSystemVersion
        return "visionOS \(version.majorVersion).\(version.minorVersion).\(version.patchVersion)"
    }()
}

enum GatewayDeviceAuthPayload {
    nonisolated static func buildV3(
        deviceId: String,
        clientId: String,
        clientMode: String,
        role: String,
        scopes: [String],
        signedAtMs: Int,
        token: String?,
        nonce: String,
        platform: String?,
        deviceFamily: String?
    ) -> String {
        [
            "v3",
            deviceId,
            clientId,
            clientMode,
            role,
            scopes.joined(separator: ","),
            String(signedAtMs),
            token ?? "",
            nonce,
            self.normalizeMetadataField(platform),
            self.normalizeMetadataField(deviceFamily),
        ].joined(separator: "|")
    }

    nonisolated static func signedDeviceDictionary(
        payload: String,
        identity: GatewayDeviceIdentity,
        signedAtMs: Int,
        nonce: String
    ) -> [String: AnyCodable]? {
        guard let signature = GatewayDeviceIdentityStore.signPayload(payload, identity: identity),
              let publicKey = GatewayDeviceIdentityStore.publicKeyBase64URL(identity)
        else {
            return nil
        }

        return [
            "id": AnyCodable(identity.deviceId),
            "publicKey": AnyCodable(publicKey),
            "signature": AnyCodable(signature),
            "signedAt": AnyCodable(signedAtMs),
            "nonce": AnyCodable(nonce),
        ]
    }

    nonisolated private static func normalizeMetadataField(_ value: String?) -> String {
        guard let value else { return "" }
        let trimmed = value.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return "" }

        var output = String()
        output.reserveCapacity(trimmed.count)
        for scalar in trimmed.unicodeScalars {
            let codePoint = scalar.value
            if codePoint >= 65, codePoint <= 90, let lowered = UnicodeScalar(codePoint + 32) {
                output.unicodeScalars.append(lowered)
            } else {
                output.unicodeScalars.append(scalar)
            }
        }
        return output
    }
}
