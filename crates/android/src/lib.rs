//! Pure-Rust Android app for the DRCX5.
//!
//! Spike 3: live video. Connects to the drone via `net::DroneLink`, decodes the
//! MJPEG stream, draws the shared `hud`, and software-blits to the screen.
//! Stays DISARMED (idle keepalive only — no motor commands); control comes next.
//!
//! Off-Android this crate is empty so the workspace still builds/tests on host.

#[cfg(target_os = "android")]
use std::mem::MaybeUninit;
#[cfg(target_os = "android")]
use std::time::{Duration, Instant};

#[cfg(target_os = "android")]
use android_activity::{AndroidApp, MainEvent, PollEvent};
#[cfg(target_os = "android")]
use ndk::{hardware_buffer_format::HardwareBufferFormat, native_window::NativeWindow};
#[cfg(target_os = "android")]
use net::{DroneLink, LinkConfig};
#[cfg(target_os = "android")]
use zune_jpeg::JpegDecoder;

#[cfg(target_os = "android")]
const VIDEO_W: usize = 240;
#[cfg(target_os = "android")]
const VIDEO_H: usize = 320;

#[cfg(target_os = "android")]
#[no_mangle]
fn android_main(app: AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("skyraptor"),
    );
    log::info!("android_main started (spike 3: live video)");

    let link = match DroneLink::start(LinkConfig::default()) {
        Ok(l) => l,
        Err(e) => {
            log::error!("DroneLink failed: {e}");
            return;
        }
    };

    let mut quit = false;
    let mut window: Option<NativeWindow> = None;
    let mut fb = vec![0u32; VIDEO_W * VIDEO_H]; // 0x00RRGGBB scratch buffer
    let mut frames: u32 = 0;
    let mut connected = false;
    let mut shown_fps = 0u32;
    let mut fps_count = 0u32;
    let mut fps_since = Instant::now();

    while !quit {
        let mut got_window = false;
        let mut lost_window = false;
        app.poll_events(Some(Duration::from_millis(16)), |event| match event {
            PollEvent::Main(MainEvent::InitWindow { .. }) => got_window = true,
            PollEvent::Main(MainEvent::TerminateWindow { .. }) => lost_window = true,
            PollEvent::Main(MainEvent::Destroy) => quit = true,
            _ => {}
        });
        if got_window {
            window = app.native_window();
            if let Some(nw) = &window {
                let _ = nw.set_buffers_geometry(
                    VIDEO_W as i32,
                    VIDEO_H as i32,
                    Some(HardwareBufferFormat::R8G8B8A8_UNORM),
                );
                log::info!("window acquired ({}x{})", nw.width(), nw.height());
            }
        }
        if lost_window {
            window = None;
        }

        // Pull the freshest decoded frame.
        let mut latest = None;
        while let Ok(f) = link.frames.try_recv() {
            latest = Some(f);
        }
        if let Some(jpeg) = latest {
            if decode_into(&jpeg, &mut fb) {
                connected = true;
                fps_count += 1;
            }
        }
        if fps_since.elapsed() >= Duration::from_secs(1) {
            shown_fps = fps_count;
            fps_count = 0;
            fps_since = Instant::now();
        }

        // HUD overlay on the scratch buffer.
        {
            let mut c = hud::Canvas { buf: &mut fb, w: VIDEO_W, h: VIDEO_H };
            let (txt, col) = if connected {
                ("LINK", hud::GREEN)
            } else {
                ("NO SIGNAL", hud::AMBER)
            };
            c.panel(2, 2, VIDEO_W - 4, 22, 150);
            c.glow_text(5, 4, txt, col, 1);
            c.glow_text(120, 4, &format!("FPS{shown_fps:02}"), hud::CYAN, 1);
            c.glow_text(5, 13, "[STANDBY]", hud::AMBER, 1);
            c.neon_frame(if connected { hud::GREEN } else { hud::RED });
        }

        if let Some(nw) = &window {
            blit(nw, &fb);
        }
        frames = frames.wrapping_add(1);
        if frames % 120 == 0 {
            log::info!("alive: connected={connected} fps={shown_fps}");
        }
    }
    link.stop();
    log::info!("android_main exiting");
}

/// Decode a JPEG frame into the `0x00RRGGBB` buffer. Returns false on size mismatch.
#[cfg(target_os = "android")]
fn decode_into(jpeg: &[u8], fb: &mut [u32]) -> bool {
    let Ok(rgb) = JpegDecoder::new(jpeg).decode() else { return false };
    if rgb.len() < VIDEO_W * VIDEO_H * 3 {
        return false;
    }
    for (px, chunk) in fb.iter_mut().zip(rgb.chunks_exact(3)) {
        *px = ((chunk[0] as u32) << 16) | ((chunk[1] as u32) << 8) | chunk[2] as u32;
    }
    true
}

/// Software-blit the `0x00RRGGBB` buffer to the window as RGBA_8888.
#[cfg(target_os = "android")]
fn blit(nw: &NativeWindow, fb: &[u32]) {
    let Ok(mut guard) = nw.lock(None) else { return };
    let (w, h, stride) = (guard.width().min(VIDEO_W), guard.height().min(VIDEO_H), guard.stride());
    let Some(bytes) = guard.bytes() else { return };
    for y in 0..h {
        for x in 0..w {
            let px = fb[y * VIDEO_W + x];
            let i = (y * stride + x) * 4;
            bytes[i] = MaybeUninit::new((px >> 16) as u8); // R
            bytes[i + 1] = MaybeUninit::new((px >> 8) as u8); // G
            bytes[i + 2] = MaybeUninit::new(px as u8); // B
            bytes[i + 3] = MaybeUninit::new(0xFF); // A
        }
    }
}
