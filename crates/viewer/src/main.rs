//! Desktop live viewer for the DRCX5.
//!
//! Shows the live MJPEG feed and flies the drone with keyboard or gamepad.
//! Starts DISARMED; the net layer only sends active commands while armed, and a
//! staleness failsafe lands the drone if this viewer hangs.
//!
//! ⚠️ When armed, the motors WILL spin. Test with PROPELLERS OFF first.
//!
//! Keyboard:
//!   Enter  arm / disarm           Space  EMERGENCY stop (cuts motors)
//!   W / S  throttle up / down     A / D  yaw left / right
//!   ↑ / ↓  pitch fwd / back       ← / →  roll left / right
//!   T takeoff   G land   C calibrate   H headless   F flip   Esc quit
//!
//! Gamepad: left stick = throttle/yaw, right stick = roll/pitch,
//!   South=takeoff, East=land, North=flip, Start=arm toggle, Select=emergency.

use std::time::{Duration, Instant};

use gilrs::{Axis, Button, Gilrs};
use minifb::{Key, KeyRepeat, Scale, Window, WindowOptions};
use net::{DroneLink, LinkConfig};
use protocol::{
    axis_to_byte, ControlState, CENTER, FLAG_CALIBRATE, FLAG_EMERGENCY, FLAG_FLIP, FLAG_HEADLESS,
    FLAG_LAND, FLAG_TAKEOFF,
};
use zune_jpeg::JpegDecoder;

const W: usize = 240;
const H: usize = 320;
/// Keyboard axis deflection from center when a key is held.
const KEY_STEP: i32 = 64;
const DEADZONE: f32 = 0.12;

/// Saturating offset from center by `delta` (positive = up).
fn offset(delta: i32) -> u8 {
    (CENTER as i32 + delta).clamp(0, 255) as u8
}

fn main() {
    let link = DroneLink::start(LinkConfig::default()).expect("start link");
    let mut gilrs = Gilrs::new().ok();

    let mut window = Window::new(
        "Skyraptor — DRCX5 [DISARMED]",
        W,
        H,
        WindowOptions { scale: Scale::X2, ..WindowOptions::default() },
    )
    .expect("open window");
    window.set_target_fps(60);

    let mut frame_buf: Vec<u32> = vec![0; W * H];
    let mut armed = false;
    let mut headless = false;
    let mut last_status = Instant::now();
    let mut fps_count = 0u32;
    let mut fps_since = Instant::now();
    let mut shown_fps = 0.0;

    println!("viewer: DISARMED. Enter=arm  Space=EMERGENCY.  ⚠ props OFF for first test.");

    while window.is_open() && !window.is_key_down(Key::Escape) {
        // --- pump gamepad events so state is current ---
        if let Some(g) = gilrs.as_mut() {
            while g.next_event().is_some() {}
        }
        let pad = gilrs.as_ref().and_then(|g| g.gamepads().next().map(|(_, gp)| gp));

        // --- edge-triggered actions (keyboard + gamepad) ---
        let pressed = window.get_keys_pressed(KeyRepeat::No);
        let pad_pressed = |b: Button| pad.map(|p| p.is_pressed(b)).unwrap_or(false);
        if pressed.contains(&Key::Enter) {
            armed = !armed;
            if armed { link.control.arm() } else { link.control.disarm() }
            window.set_title(if armed {
                "Skyraptor — DRCX5 [ARMED]"
            } else {
                "Skyraptor — DRCX5 [DISARMED]"
            });
        }
        if pressed.contains(&Key::H) {
            headless = !headless;
        }

        // --- build control state from held keys + gamepad axes ---
        let mut roll = CENTER;
        let mut pitch = CENTER;
        let mut throttle = CENTER;
        let mut yaw = CENTER;

        // keyboard (held)
        let down = |k: Key| window.is_key_down(k);
        if down(Key::W) { throttle = offset(KEY_STEP); }
        if down(Key::S) { throttle = offset(-KEY_STEP); }
        if down(Key::D) { yaw = offset(KEY_STEP); }
        if down(Key::A) { yaw = offset(-KEY_STEP); }
        if down(Key::Right) { roll = offset(KEY_STEP); }
        if down(Key::Left) { roll = offset(-KEY_STEP); }
        if down(Key::Up) { pitch = offset(KEY_STEP); }
        if down(Key::Down) { pitch = offset(-KEY_STEP); }

        // gamepad (overrides axis if pushed past deadzone)
        if let Some(p) = pad {
            let ax = |a: Axis| p.value(a);
            let apply = |cur: u8, v: f32| {
                if v.abs() > DEADZONE { axis_to_byte(v) } else { cur }
            };
            throttle = apply(throttle, ax(Axis::LeftStickY)); // stick up = climb
            yaw = apply(yaw, ax(Axis::LeftStickX));
            roll = apply(roll, ax(Axis::RightStickX));
            pitch = apply(pitch, ax(Axis::RightStickY));
        }

        // flags
        let mut flags = 0u8;
        if headless { flags |= FLAG_HEADLESS; }
        if down(Key::T) || pad_pressed(Button::South) { flags |= FLAG_TAKEOFF; }
        if down(Key::G) || pad_pressed(Button::East) { flags |= FLAG_LAND; }
        if down(Key::C) { flags |= FLAG_CALIBRATE; }
        if down(Key::F) || pad_pressed(Button::North) { flags |= FLAG_FLIP; }

        // EMERGENCY: force-arm so the cut actually transmits, then drop armed.
        let emergency = window.is_key_down(Key::Space) || pad_pressed(Button::Select);
        if emergency {
            flags |= FLAG_EMERGENCY;
            link.control.arm();
            armed = false; // user must re-arm with Enter
            window.set_title("Skyraptor — DRCX5 [EMERGENCY]");
        }

        link.control.set(ControlState { roll, pitch, throttle, yaw, flags });
        if !armed && !emergency {
            link.control.disarm();
        }

        // --- pull the freshest video frame and decode it ---
        let mut latest = None;
        while let Ok(f) = link.frames.try_recv() {
            latest = Some(f);
        }
        if let Some(jpeg) = latest {
            if let Ok(rgb) = JpegDecoder::new(&jpeg).decode() {
                // RGB8 -> 0x00RRGGBB; only blit if dimensions match the window.
                if rgb.len() >= W * H * 3 {
                    for (i, px) in frame_buf.iter_mut().enumerate() {
                        let r = rgb[i * 3] as u32;
                        let g = rgb[i * 3 + 1] as u32;
                        let b = rgb[i * 3 + 2] as u32;
                        *px = (r << 16) | (g << 8) | b;
                    }
                    fps_count += 1;
                }
            }
        }

        // status border: green armed / red disarmed
        let border = if armed { 0x00_00FF00 } else { 0x00_FF0000 };
        for x in 0..W {
            frame_buf[x] = border;
            frame_buf[(H - 1) * W + x] = border;
        }
        for y in 0..H {
            frame_buf[y * W] = border;
            frame_buf[y * W + (W - 1)] = border;
        }

        window.update_with_buffer(&frame_buf, W, H).expect("blit");

        // ~1 Hz fps + status to terminal
        if fps_since.elapsed() >= Duration::from_secs(1) {
            shown_fps = fps_count as f32;
            fps_count = 0;
            fps_since = Instant::now();
        }
        if last_status.elapsed() >= Duration::from_millis(500) {
            println!(
                "[{}] thr={throttle:3} yaw={yaw:3} roll={roll:3} pitch={pitch:3} flags={flags:02x}  {shown_fps:.0} fps",
                if armed { "ARMED" } else { "disarmed" }
            );
            last_status = Instant::now();
        }
    }

    link.stop();
    println!("viewer: stopped.");
}
