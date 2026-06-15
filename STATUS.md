# Skyraptor — status

Pure-Rust control + video for the Vivitar Sky Raptor **DRCX5** toy drone
(stock app: the stock app, `the stock app`).

## Done

- **Phase 0 — Protocol discovery** ✅
  Implemented from the the stock app APK and confirmed against a live packet
  capture. Control: UDP `192.168.4.153:8090`, 8-byte `66…99` (throttle centered
  at `0x80`, altitude-hold). Video: UDP `:8080`, MJPEG (`TZH`-headered chunks),
  240×320. See `docs/superpowers/specs/2026-06-13-drcx5-protocol-spec.md`.

- **Phase 1 — Desktop app** ✅ (validated props-off against the real drone)
  - `crates/protocol` — control encoding + checksum + MJPEG reassembly + axis mapping (22 tests)
  - `crates/net` — ~25 Hz control sender with self-neutralizing failsafe + video receiver (6 tests)
  - `crates/skyctl` — live connectivity probe (video/FPS, disarmed)
  - `crates/viewer` — live MJPEG window + keyboard/gamepad flight control, with
    on-screen HUD, trim, expo/rate-limited control feel, and snapshot/recording
  - 33 tests total, clippy clean.

- **Phase 2 — Android app** ✅ (video + gamepad control working on-device)
  - `crates/hud` — shared cyberpunk HUD renderer (desktop + android)
  - `crates/android` — pure-Rust `android-activity` app; software-blits MJPEG to
    the NativeWindow, reuses `protocol`/`net`/`hud`. Live video (~21 fps) +
    gamepad flight control + same failsafe/HUD as desktop.
  - **WiFi bind:** JNI `ConnectivityManager.bindProcessToNetwork` so video works
    with cellular on (Android otherwise routes app traffic to cellular).
  - Gamepad map documented in `docs/CONTROLLER.md`; key events consumed so a
    BACK-mapped button can't close the app.
  - Toolchain: `cargo-apk` + NDK r27, 16 KB-aligned. Build: `scripts/build-android.sh`.

## Status: maiden flight flown ✅

Android app flew the drone with live video + gamepad control. Confirmed:
- Video (native-res, crisp HUD, safe-area inset for Pixel cutout/corners).
- Gamepad: left stick X/Y = yaw/throttle, right Z/Rz = roll/pitch; Start=arm,
  B=takeoff, A=land, X=calibrate, Y=flip, Select/Mode=EMERGENCY.
- WiFi-bind (works with cellular on), self-healing watchdog (re-binds when
  disarmed), keep-screen-on, LINK-LOST warning, crisp native-res HUD.
- **Tap RECONNECT to join** the drone AP via `WifiNetworkSpecifier` (SSID prefix
  `WIFI_8K__`) — shows the one-time system Connect dialog. The join is tap-only:
  the watchdog silently re-binds/restarts the link when already on the AP but
  never prompts on its own, so shutting the drone off (e.g. to use the sim)
  doesn't pop the WiFi menu.
- **Maiden flight ended by WiFi range (RSSI -84) — a hardware limit. Fly close.**

### One-time setup on a fresh install
- Grant **NEARBY_WIFI_DEVICES** (Settings → Apps → Skyraptor → Permissions →
  Nearby devices, or `adb shell pm grant app.skyraptor.drcx5
  android.permission.NEARBY_WIFI_DEVICES`) — needed for the auto-connect WiFi
  scan. Can't be requested in-app (pure-Rust NativeActivity has no Activity
  handle / UI thread for the runtime-permission dialog).

## Android extras (done)

- **Link-quality meter** (fps-based bars, green/amber/red) — range warning.
- **Trim** on the D-pad, **headless** on L1, **trim-reset** on R1; shown in HUD.
- **Touch RECONNECT** button (shown when disconnected) — the only thing that
  pops the system WiFi-join dialog; never auto-prompts on signal loss.
- **KEY MAP settings** (disarmed): view + remap any action to any button by
  pressing it; persisted to `bindings.txt`. Input is data-driven.
- **SIM** (disarmed): fly a neon wireframe virtual drone with your mappings to
  preview/practice — nothing sent to the real drone.
- One-time setup unchanged: grant NEARBY_WIFI_DEVICES for the RECONNECT scan.

## Next

- More close-range flights; tune EXPO/MAX_DEFLECTION.
- Possible: confirm a faint doubled-text artifact in the overlay screens is only
  a screenshot/compositor effect, not on-device.

Build/deploy: `scripts/build-android.sh build` then
`adb install -r target/debug/apk/skyraptor-android.apk`. Logs: `adb logcat -s skyraptor:I`.

## Tooling notes

- Analyze a capture: `python3 scripts/analyze_pcap.py <pcap>`
- Validate reassembly on a capture: `cargo run -p protocol --example replay_pcap -- <pcap> <out>`
- Run the viewer (props OFF first): `cargo run -p viewer --release`
