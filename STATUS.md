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

## Next

- Bench-verify gamepad control on the phone (arm + sticks → motors, props off).
- Optional: wire spare buttons / 8-way D-pad (headless, trim, etc.).
- Real tethered flight test (beyond props-off bench check).

## Tooling notes

- Analyze a capture: `python3 scripts/analyze_pcap.py <pcap>`
- Validate reassembly on a capture: `cargo run -p protocol --example replay_pcap -- <pcap> <out>`
- Run the viewer (props OFF first): `cargo run -p viewer --release`
