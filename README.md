# Glitchwing

A pure-Rust neon-HUD cockpit for cheap WiFi-camera toy drones (sold under many
names; built and tested with a **Vivitar Sky Raptor DRCX5**).

Live video, full flight control from a physical gamepad, a cyberpunk heads-up
display, and a wireframe flight simulator. The protocol, networking, and HUD are
platform-agnostic Rust crates; today's flight client is **Android** (no
Java/Kotlin) with a **desktop** viewer, and the core is ready to port elsewhere.

## Features

- **Live video** — MJPEG over UDP, decoded and software-blitted at native
  resolution with an aspect-correct, safe-area-aware HUD.
- **Gamepad flight control** — roll/pitch/yaw/throttle with deadzone + expo
  shaping; arm, takeoff, land, flip (aimed by the right stick), calibrate,
  headless, and an emergency **killswitch**.
- **Cyberpunk HUD** — neon glow text, scanlines, stick boxes, throttle/trim
  readout, and an fps-based link-quality meter.
- **Speed presets** — LOW / MED / HIGH stick-deflection tiers, cycled in flight.
- **Photo + video capture** — snapshot the current frame to JPEG; record the
  live stream straight to a Motion-JPEG AVI (no re-encode).
- **D-pad trim** — roll/pitch trim with on-HUD readout.
- **KEY MAP** — remap any action to any controller button; persisted.
- **SIM** — fly a neon wireframe drone with your real mappings (disarmed,
  nothing sent) to practice and preview controls.
- **WiFi handling** — binds app traffic to the drone's local AP (works with
  cellular on), tap-the-feed to (re)join, self-healing reconnect, keep-screen-on,
  and a LINK-LOST warning with auto-suspend.

## Project layout

| Crate | Role |
|---|---|
| `crates/protocol` | Drone wire protocol: control-packet encode/checksum, MJPEG reassembly, MJPEG-in-AVI muxer (host-testable, no Android deps). |
| `crates/net` | UDP control sender (~25 Hz, self-neutralizing failsafe) + video receiver. |
| `crates/hud` | Shared cyberpunk framebuffer HUD renderer (desktop + Android). |
| `crates/android` | The Android app (`android-activity`, software-blit render, JNI WiFi). |
| `crates/viewer` | Desktop companion: live MJPEG window + keyboard/gamepad control. |
| `crates/skyctl` | Desktop connectivity probe. |

## Build & flash (Android)

Requires the Android NDK (r27) and `cargo-apk`:

```sh
scripts/build-android.sh build
adb install -r target/debug/apk/glitchwing-android.apk
```

One-time on a fresh install, grant the WiFi-scan permission (needed for the
join picker; a pure-Rust NativeActivity can't request it at runtime):

```sh
adb shell pm grant app.glitchwing android.permission.NEARBY_WIFI_DEVICES
```

Logs: `adb logcat -s glitchwing:I`.

## Desktop viewer / tests

```sh
cargo run -p viewer --release      # live window (props OFF first!)
cargo test --workspace             # protocol + net unit tests
```

## ⚠️ Safety

These are GPS-less, altitude-hold toy drones — a control bug means a crash or
flyaway, not an exception. **Always test with props off first.** The app starts
disarmed; arming spins the motors. Fly close — these drop the link at modest
WiFi range.

## License

[MIT](LICENSE) © 2026 Otis Stamp
