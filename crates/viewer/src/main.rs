//! Desktop live viewer for the DRCX5.
//!
//! Shows the live MJPEG feed and flies the drone with keyboard or gamepad.
//! Starts DISARMED; the net layer only sends active commands while armed, and a
//! staleness failsafe lands the drone if this viewer hangs.
//!
//! ⚠️ When armed, the motors WILL spin. Test with PROPELLERS OFF first.
//!
//! Flight:  Enter arm/disarm · Space EMERGENCY · W/S throttle · A/D yaw
//!          ↑/↓ pitch · ←/→ roll · T takeoff · G land · C calibrate · H headless · F flip
//! Trim:    J/L roll · I/K pitch · U/O yaw · R reset
//! Capture: P snapshot (.jpg) · V toggle recording (.mjpeg) · Esc quit
//! Gamepad: left stick throttle/yaw · right stick roll/pitch · Start arm · Select EMERGENCY

mod hud;

use std::fs::{self, File};
use std::io::Write;
use std::time::{Duration, Instant};

use gilrs::{Axis, Button, EventType, Gilrs};
use minifb::{Key, KeyRepeat, Scale, Window, WindowOptions};
use net::{DroneLink, LinkConfig};
use protocol::{
    apply_trim, axis_to_byte, expo, ramp_toward, ControlState, CENTER, FLAG_CALIBRATE,
    FLAG_EMERGENCY, FLAG_FLIP, FLAG_HEADLESS, FLAG_LAND, FLAG_TAKEOFF,
};
use zune_jpeg::JpegDecoder;

const W: usize = 240;
const H: usize = 320;
const MAX_DEFLECTION: f32 = 0.7; // fraction of full stick range (gentler control)
const EXPO: f32 = 0.4; // 0 = linear, 1 = cubic
const THROTTLE_RAMP: u8 = 6; // max throttle byte change per frame
const TRIM_LIMIT: i8 = 40;

#[derive(Default, Clone, Copy)]
struct Trim {
    roll: i8,
    pitch: i8,
    yaw: i8,
}

/// Shape a raw axis (`-1..=1`) into a protocol byte: expo + max-deflection + trim.
fn shape(raw: f32, trim: i8) -> u8 {
    apply_trim(axis_to_byte(expo(raw, EXPO) * MAX_DEFLECTION), trim)
}

fn main() {
    let link = DroneLink::start(LinkConfig::default()).expect("start link");
    let mut gilrs = Gilrs::new().ok();

    let mut window = Window::new(
        "Skyraptor — DRCX5",
        W,
        H,
        WindowOptions { scale: Scale::X2, ..WindowOptions::default() },
    )
    .expect("open window");
    window.set_target_fps(60);

    let mut buf: Vec<u32> = vec![0; W * H];
    let mut armed = false;
    let mut headless = false;
    let mut trim = Trim::default();
    let mut prev_throttle = CENTER;
    let mut last_jpeg: Option<Vec<u8>> = None;
    let mut recorder: Option<File> = None;
    let (mut snap_n, mut rec_n) = (0u32, 0u32);
    let (mut fps_count, mut shown_fps) = (0u32, 0u32);
    let mut fps_since = Instant::now();

    fs::create_dir_all("snapshots").ok();
    fs::create_dir_all("recordings").ok();
    println!("viewer: DISARMED. Enter=arm  Space=EMERGENCY.  ⚠ props OFF for first test.");

    while window.is_open() && !window.is_key_down(Key::Escape) {
        // --- gamepad: pump events, capture Start (arm) edge, read axis state ---
        let mut pad_arm_edge = false;
        if let Some(g) = gilrs.as_mut() {
            while let Some(ev) = g.next_event() {
                if let EventType::ButtonPressed(Button::Start, _) = ev.event {
                    pad_arm_edge = true;
                }
            }
        }
        let pad = gilrs.as_ref().and_then(|g| g.gamepads().next().map(|(_, gp)| gp));
        let pad_pressed = |b: Button| pad.map(|p| p.is_pressed(b)).unwrap_or(false);

        // --- edge-triggered keys ---
        let pressed = window.get_keys_pressed(KeyRepeat::No);
        let hit = |k: Key| pressed.contains(&k);

        if hit(Key::Enter) || pad_arm_edge {
            armed = !armed;
            if armed {
                link.control.arm();
            } else {
                link.control.disarm();
            }
        }
        if hit(Key::H) {
            headless = !headless;
        }
        // trim adjustments
        if hit(Key::L) { trim.roll = (trim.roll + 1).min(TRIM_LIMIT); }
        if hit(Key::J) { trim.roll = (trim.roll - 1).max(-TRIM_LIMIT); }
        if hit(Key::I) { trim.pitch = (trim.pitch + 1).min(TRIM_LIMIT); }
        if hit(Key::K) { trim.pitch = (trim.pitch - 1).max(-TRIM_LIMIT); }
        if hit(Key::O) { trim.yaw = (trim.yaw + 1).min(TRIM_LIMIT); }
        if hit(Key::U) { trim.yaw = (trim.yaw - 1).max(-TRIM_LIMIT); }
        if hit(Key::R) { trim = Trim::default(); }

        // --- raw axes from keyboard, overridden by gamepad past the deadzone ---
        let down = |k: Key| window.is_key_down(k);
        let key_axis = |neg: bool, pos: bool| match (neg, pos) {
            (false, true) => 1.0,
            (true, false) => -1.0,
            _ => 0.0,
        };
        let mut thr_raw = key_axis(down(Key::S), down(Key::W));
        let mut yaw_raw = key_axis(down(Key::A), down(Key::D));
        let mut roll_raw = key_axis(down(Key::Left), down(Key::Right));
        let mut pitch_raw = key_axis(down(Key::Down), down(Key::Up));
        if let Some(p) = pad {
            let pick = |raw: f32, v: f32| if v.abs() > 0.12 { v } else { raw };
            thr_raw = pick(thr_raw, p.value(Axis::LeftStickY));
            yaw_raw = pick(yaw_raw, p.value(Axis::LeftStickX));
            roll_raw = pick(roll_raw, p.value(Axis::RightStickX));
            pitch_raw = pick(pitch_raw, p.value(Axis::RightStickY));
        }

        let roll = shape(roll_raw, trim.roll);
        let pitch = shape(pitch_raw, trim.pitch);
        let yaw = shape(yaw_raw, trim.yaw);
        // throttle: shaped target, rate-limited; reset to center while disarmed.
        let thr_target = axis_to_byte(expo(thr_raw, EXPO) * MAX_DEFLECTION);
        let throttle = if armed {
            ramp_toward(prev_throttle, thr_target, THROTTLE_RAMP)
        } else {
            CENTER
        };
        prev_throttle = throttle;

        // flags
        let mut flags = 0u8;
        if headless { flags |= FLAG_HEADLESS; }
        if down(Key::T) || pad_pressed(Button::South) { flags |= FLAG_TAKEOFF; }
        if down(Key::G) || pad_pressed(Button::East) { flags |= FLAG_LAND; }
        if down(Key::C) { flags |= FLAG_CALIBRATE; }
        if down(Key::F) || pad_pressed(Button::North) { flags |= FLAG_FLIP; }

        let emergency = window.is_key_down(Key::Space) || pad_pressed(Button::Select);
        if emergency {
            flags |= FLAG_EMERGENCY;
            link.control.arm(); // force-transmit the cut
            armed = false;
        }

        link.control.set(ControlState { roll, pitch, throttle, yaw, flags });
        if !armed && !emergency {
            link.control.disarm();
        }

        // --- freshest video frame: keep raw bytes, decode for display ---
        let mut latest = None;
        while let Ok(f) = link.frames.try_recv() {
            latest = Some(f);
        }
        if let Some(jpeg) = latest {
            if let Ok(rgb) = JpegDecoder::new(&jpeg).decode() {
                if rgb.len() >= W * H * 3 {
                    for (i, px) in buf.iter_mut().enumerate() {
                        let (r, g, b) = (rgb[i * 3] as u32, rgb[i * 3 + 1] as u32, rgb[i * 3 + 2] as u32);
                        *px = (r << 16) | (g << 8) | b;
                    }
                    fps_count += 1;
                }
            }
            if let Some(f) = recorder.as_mut() {
                let _ = f.write_all(&jpeg);
            }
            last_jpeg = Some(jpeg);
        }

        // --- capture: snapshot / record toggle ---
        if hit(Key::P) {
            if let Some(j) = &last_jpeg {
                let path = format!("snapshots/snap_{snap_n:03}.jpg");
                if fs::write(&path, j).is_ok() {
                    println!("snapshot -> {path}");
                    snap_n += 1;
                }
            }
        }
        if hit(Key::V) {
            if recorder.is_some() {
                recorder = None;
                println!("recording stopped");
            } else {
                let path = format!("recordings/rec_{rec_n:03}.mjpeg");
                match File::create(&path) {
                    Ok(f) => {
                        println!("recording -> {path} (concatenated JPEGs)");
                        recorder = Some(f);
                        rec_n += 1;
                    }
                    Err(e) => eprintln!("record failed: {e}"),
                }
            }
        }

        // status border: green armed / red disarmed
        let border = if armed { 0x0000_FF00 } else { 0x00FF_0000 };
        for x in 0..W {
            buf[x] = border;
            buf[(H - 1) * W + x] = border;
        }
        for y in 0..H {
            buf[y * W] = border;
            buf[y * W + (W - 1)] = border;
        }

        // --- HUD ---
        if fps_since.elapsed() >= Duration::from_secs(1) {
            shown_fps = fps_count;
            fps_count = 0;
            fps_since = Instant::now();
        }
        let white = 0x00FF_FFFF;
        let mut canvas = hud::Canvas { buf: &mut buf, w: W, h: H };
        let (state_txt, state_col) = if armed { ("ARMED", 0x0000_FF00) } else { ("DISARMED", 0x00FF_4040) };
        canvas.text(4, 4, state_txt, state_col, 1);
        canvas.text(4, 14, &format!("THR {throttle:3} YAW {yaw:3}"), white, 1);
        canvas.text(4, 24, &format!("ROL {roll:3} PIT {pitch:3}"), white, 1);
        canvas.text(4, 34, &format!("TRIM R{:+} P{:+} Y{:+}", trim.roll, trim.pitch, trim.yaw), white, 1);
        canvas.text(4, 44, &format!("FPS {shown_fps} FLG {flags:02X}"), white, 1);
        if recorder.is_some() {
            canvas.text(4, 54, "REC", 0x00FF_0000, 1);
        }

        window.update_with_buffer(&buf, W, H).expect("blit");
    }

    link.stop();
    println!("viewer: stopped.");
}
