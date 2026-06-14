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
  - `crates/viewer` — live MJPEG window + keyboard/gamepad flight control
  - 28 tests total, clippy clean.

## Next

- Real tethered flight test (beyond props-off bench check).
- Android port: `android-activity` + `cargo-ndk` toolchain spike, then reuse
  `protocol`/`net` and add a wgpu/egui video+HUD + GameActivity gamepad input.

## Tooling notes

- Analyze a capture: `python3 scripts/analyze_pcap.py <pcap>`
- Validate reassembly on a capture: `cargo run -p protocol --example replay_pcap -- <pcap> <out>`
- Run the viewer (props OFF first): `cargo run -p viewer --release`
