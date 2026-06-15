//! Pure-Rust Android app for the DRCX5: live MJPEG video + gamepad flight control.
//!
//! Reuses `protocol` (encoding + input shaping), `net::DroneLink` (sender with
//! failsafe + video receiver), and the shared `hud`. Renders by software-blitting
//! to the NativeWindow. Gamepad via `android-activity` input events.
//!
//! ⚠️ Starts DISARMED. When armed the motors WILL spin — test with PROPS OFF first.
//! Gamepad: left stick throttle/yaw · right stick roll/pitch · Start arm ·
//! Select/Mode EMERGENCY · A takeoff · B land · Y flip · X calibrate.
//!
//! Off-Android this crate is empty so the workspace still builds/tests on host.

#[cfg(target_os = "android")]
mod app;
#[cfg(target_os = "android")]
mod settings;

#[cfg(target_os = "android")]
#[no_mangle]
fn android_main(app: android_activity::AndroidApp) {
    app::run(app);
}
