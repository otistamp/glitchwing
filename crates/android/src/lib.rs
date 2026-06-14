//! Pure-Rust Android app for the DRCX5.
//!
//! Spike 2: software-blit an animated gradient to the NativeWindow, proving the
//! exact render path we'll use for video (lock buffer → write RGBA → post).
//! Verify visually on the phone and via `adb logcat -s skyraptor`.
//!
//! Off-Android this crate is empty so the workspace still builds/tests on host.

#[cfg(target_os = "android")]
use std::mem::MaybeUninit;

#[cfg(target_os = "android")]
use android_activity::{AndroidApp, MainEvent, PollEvent};
#[cfg(target_os = "android")]
use ndk::{hardware_buffer_format::HardwareBufferFormat, native_window::NativeWindow};

#[cfg(target_os = "android")]
#[no_mangle]
fn android_main(app: AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("skyraptor"),
    );
    log::info!("android_main started (spike 2: render)");

    let mut quit = false;
    let mut window: Option<NativeWindow> = None;
    let mut frame: u32 = 0;

    while !quit {
        let mut got_window = false;
        let mut lost_window = false;
        app.poll_events(Some(std::time::Duration::from_millis(16)), |event| match event {
            PollEvent::Main(MainEvent::InitWindow { .. }) => got_window = true,
            PollEvent::Main(MainEvent::TerminateWindow { .. }) => lost_window = true,
            PollEvent::Main(MainEvent::Destroy) => quit = true,
            _ => {}
        });

        if got_window {
            window = app.native_window();
            if let Some(nw) = &window {
                // Force RGBA_8888; keep the window's native size (0,0).
                let _ = nw.set_buffers_geometry(0, 0, Some(HardwareBufferFormat::R8G8B8A8_UNORM));
                log::info!("window {}x{} acquired", nw.width(), nw.height());
            }
        }
        if lost_window {
            window = None;
        }

        if let Some(nw) = &window {
            render(nw, frame);
            frame = frame.wrapping_add(2);
        }
    }
    log::info!("android_main exiting");
}

/// Fill the window with an animated gradient (RGBA_8888), respecting stride.
#[cfg(target_os = "android")]
fn render(nw: &NativeWindow, frame: u32) {
    let mut guard = match nw.lock(None) {
        Ok(g) => g,
        Err(_) => return,
    };
    let (w, h, stride) = (guard.width(), guard.height(), guard.stride());
    let Some(bytes) = guard.bytes() else { return };
    let f = frame as usize;
    for y in 0..h {
        for x in 0..w {
            let i = (y * stride + x) * 4;
            bytes[i] = MaybeUninit::new(((x + f) & 0xFF) as u8); // R
            bytes[i + 1] = MaybeUninit::new(((y + f) & 0xFF) as u8); // G
            bytes[i + 2] = MaybeUninit::new((f & 0xFF) as u8); // B
            bytes[i + 3] = MaybeUninit::new(0xFF); // A
        }
    }
    // guard drops here -> unlock + post
}
