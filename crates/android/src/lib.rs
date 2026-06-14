//! Pure-Rust Android app for the DRCX5.
//!
//! Spike 1: prove the toolchain — the app starts, `android_main` runs, and we
//! log lifecycle events (verify with `adb logcat -s skyraptor`). Rendering,
//! networking and input come next.
//!
//! Off-Android this crate is empty so the workspace still builds/tests on host.

#[cfg(target_os = "android")]
use android_activity::{AndroidApp, MainEvent, PollEvent};

#[cfg(target_os = "android")]
#[no_mangle]
fn android_main(app: AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("skyraptor"),
    );
    log::info!("android_main started — toolchain spike alive");

    let mut quit = false;
    let mut frames: u64 = 0;
    while !quit {
        app.poll_events(Some(std::time::Duration::from_millis(500)), |event| match event {
            PollEvent::Main(MainEvent::InitWindow { .. }) => log::info!("InitWindow"),
            PollEvent::Main(MainEvent::TerminateWindow { .. }) => log::info!("TerminateWindow"),
            PollEvent::Main(MainEvent::GainedFocus) => log::info!("GainedFocus"),
            PollEvent::Main(MainEvent::LostFocus) => log::info!("LostFocus"),
            PollEvent::Main(MainEvent::Destroy) => {
                log::info!("Destroy");
                quit = true;
            }
            _ => {}
        });
        frames += 1;
        if frames % 4 == 0 {
            log::info!("heartbeat {frames}");
        }
    }
    log::info!("android_main exiting");
}
