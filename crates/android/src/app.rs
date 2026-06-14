//! Android app loop: input → control, video → screen. Android-only.

use std::mem::MaybeUninit;
use std::time::{Duration, Instant};

use android_activity::input::{Axis, InputEvent, KeyAction, Keycode};
use android_activity::{AndroidApp, InputStatus, MainEvent, PollEvent};
use ndk::{hardware_buffer_format::HardwareBufferFormat, native_window::NativeWindow};
use net::{DroneLink, LinkConfig};
use protocol::{
    axis_to_byte, expo, ramp_toward, ControlState, CENTER, FLAG_CALIBRATE, FLAG_EMERGENCY,
    FLAG_FLIP, FLAG_LAND, FLAG_TAKEOFF,
};
use zune_jpeg::JpegDecoder;

const VIDEO_W: usize = 240;
const VIDEO_H: usize = 320;
const MAX_DEFLECTION: f32 = 0.7;
const EXPO: f32 = 0.4;
const THROTTLE_RAMP: u8 = 6;
const DEADZONE: f32 = 0.12;
/// Android joystick "class" bit in a MotionEvent source (SOURCE_CLASS_JOYSTICK).
const SOURCE_CLASS_JOYSTICK: u32 = 0x0000_0010;

/// Latest gamepad state, accumulated from input events.
#[derive(Default)]
struct Pad {
    lx: f32,
    ly: f32,
    rx: f32,
    ry: f32,
    arm_toggle: bool, // one-shot edge (Start pressed)
    emergency: bool,
    takeoff: bool,
    land: bool,
    flip: bool,
    calibrate: bool,
    last_key: u32, // last gamepad keycode seen (debug)
}

pub fn run(app: AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("skyraptor"),
    );
    log::info!("android_main (spike 4: gamepad control)");

    let link = match DroneLink::start(LinkConfig::default()) {
        Ok(l) => l,
        Err(e) => {
            log::error!("DroneLink failed: {e}");
            return;
        }
    };

    let mut quit = false;
    let mut window: Option<NativeWindow> = None;
    let mut fb = vec![0u32; VIDEO_W * VIDEO_H];
    let mut pad = Pad::default();
    let mut armed = false;
    let mut prev_throttle = CENTER;
    let mut connected = false;
    let (mut fps_count, mut shown_fps) = (0u32, 0u32);
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
            }
        }
        if lost_window {
            window = None;
        }

        // Drain input events into `pad`.
        if let Ok(mut iter) = app.input_events_iter() {
            while iter.next(|e| handle_input(e, &mut pad)) {}
        }

        // Arming edge.
        if pad.arm_toggle {
            armed = !armed;
            if armed {
                link.control.arm();
            } else {
                link.control.disarm();
            }
            pad.arm_toggle = false;
        }

        // Axes → bytes (Android stick Y is inverted: up = -1, so negate for climb/forward).
        let dz = |v: f32| if v.abs() > DEADZONE { v } else { 0.0 };
        let shape = |raw: f32| axis_to_byte(expo(raw, EXPO) * MAX_DEFLECTION);
        let roll = shape(dz(pad.rx));
        let pitch = shape(dz(-pad.ry));
        let yaw = shape(dz(pad.lx));
        let throttle = if armed {
            prev_throttle = ramp_toward(prev_throttle, shape(dz(-pad.ly)), THROTTLE_RAMP);
            prev_throttle
        } else {
            prev_throttle = CENTER;
            CENTER
        };

        let mut flags = 0u8;
        if pad.takeoff {
            flags |= FLAG_TAKEOFF;
        }
        if pad.land {
            flags |= FLAG_LAND;
        }
        if pad.flip {
            flags |= FLAG_FLIP;
        }
        if pad.calibrate {
            flags |= FLAG_CALIBRATE;
        }
        if pad.emergency {
            flags |= FLAG_EMERGENCY;
            link.control.arm(); // force-transmit the cut
            armed = false;
        }
        link.control.set(ControlState { roll, pitch, throttle, yaw, flags });
        if !armed && !pad.emergency {
            link.control.disarm();
        }

        // Freshest video frame.
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

        let dbg = format!(
            "LX{:+.2} LY{:+.2} RX{:+.2} RY{:+.2} K{}",
            pad.lx, pad.ly, pad.rx, pad.ry, pad.last_key
        );
        draw_hud(&mut fb, armed, connected, shown_fps, throttle, yaw, roll, pitch, flags, &dbg);

        if let Some(nw) = &window {
            blit(nw, &fb);
        }
    }
    link.stop();
    log::info!("android_main exiting");
}

fn handle_input(event: &InputEvent, pad: &mut Pad) -> InputStatus {
    match event {
        InputEvent::MotionEvent(m) => {
            if u32::from(m.source()) & SOURCE_CLASS_JOYSTICK != 0 && m.pointer_count() > 0 {
                let p = m.pointer_at_index(0);
                pad.lx = p.axis_value(Axis::X);
                pad.ly = p.axis_value(Axis::Y);
                pad.rx = p.axis_value(Axis::Z);
                pad.ry = p.axis_value(Axis::Rz);
                return InputStatus::Handled;
            }
            InputStatus::Unhandled
        }
        InputEvent::KeyEvent(k) => {
            let down = matches!(k.action(), KeyAction::Down);
            if down {
                pad.last_key = u32::from(k.key_code());
            }
            match k.key_code() {
                Keycode::ButtonStart => {
                    if down && k.repeat_count() == 0 {
                        pad.arm_toggle = true;
                    }
                }
                Keycode::ButtonSelect | Keycode::ButtonMode => pad.emergency = down,
                Keycode::ButtonA => pad.takeoff = down,
                Keycode::ButtonB => pad.land = down,
                Keycode::ButtonY => pad.flip = down,
                Keycode::ButtonX => pad.calibrate = down,
                _ => {}
            }
            // Consume ALL key events so a controller button mapped to BACK can't
            // finish the activity (the default fallback for unhandled BACK).
            InputStatus::Handled
        }
        _ => InputStatus::Unhandled,
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_hud(
    fb: &mut [u32],
    armed: bool,
    connected: bool,
    fps: u32,
    throttle: u8,
    yaw: u8,
    roll: u8,
    pitch: u8,
    flags: u8,
    dbg: &str,
) {
    let mut c = hud::Canvas { buf: fb, w: VIDEO_W, h: VIDEO_H };
    c.neon_frame(if armed { hud::GREEN } else { hud::RED });
    c.panel(2, 2, VIDEO_W - 4, 44, 150);
    let (txt, col) = if armed { ("[ARMED]", hud::GREEN) } else { ("[STANDBY]", hud::AMBER) };
    c.glow_text(5, 4, txt, col, 1);
    let link = if connected { "LINK" } else { "NO SIG" };
    c.glow_text(96, 4, link, if connected { hud::CYAN } else { hud::AMBER }, 1);
    c.glow_text(168, 4, &format!("FPS{fps:02}"), hud::CYAN, 1);
    c.glow_text(5, 15, "THR", hud::CYAN, 1);
    c.bar(34, 15, VIDEO_W - 44, 7, throttle as f32 / 255.0, if armed { hud::GREEN } else { hud::AMBER });
    c.glow_text(5, 26, &format!("FLG{flags:02X}"), hud::MAGENTA, 1);
    c.glow_text(5, 37, dbg, hud::GREEN, 1); // raw gamepad debug

    let bs = 46;
    c.stick_box(8, VIDEO_H - bs - 12, bs, yaw, throttle, hud::MAGENTA);
    c.glow_text(8, VIDEO_H - 10, "YAW/THR", hud::MAGENTA, 1);
    c.stick_box(VIDEO_W - bs - 9, VIDEO_H - bs - 12, bs, roll, pitch, hud::CYAN);
    c.glow_text(VIDEO_W - 64, VIDEO_H - 10, "ROL/PIT", hud::CYAN, 1);
}

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

fn blit(nw: &NativeWindow, fb: &[u32]) {
    let Ok(mut guard) = nw.lock(None) else { return };
    let (w, h, stride) = (guard.width().min(VIDEO_W), guard.height().min(VIDEO_H), guard.stride());
    let Some(bytes) = guard.bytes() else { return };
    for y in 0..h {
        for x in 0..w {
            let px = fb[y * VIDEO_W + x];
            let i = (y * stride + x) * 4;
            bytes[i] = MaybeUninit::new((px >> 16) as u8);
            bytes[i + 1] = MaybeUninit::new((px >> 8) as u8);
            bytes[i + 2] = MaybeUninit::new(px as u8);
            bytes[i + 3] = MaybeUninit::new(0xFF);
        }
    }
}
