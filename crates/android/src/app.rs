//! Android app loop: input → control, video → screen. Android-only.
//!
//! Renders at the phone's NATIVE resolution: the video is decoded at 240x320 and
//! scaled (aspect-correct, centered) into the framebuffer, and the HUD is drawn
//! crisp at full res inside a safe-area inset (clears rounded corners + camera
//! cutout). Reuses `protocol`/`net`/`hud`. Stays disarmed until armed via Start.

use std::mem::MaybeUninit;
use std::time::{Duration, Instant};

use android_activity::input::{Axis, InputEvent, KeyAction, Keycode};
use android_activity::{AndroidApp, InputStatus, MainEvent, PollEvent};
use jni::objects::{JObject, JObjectArray, JValue};
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
const SOURCE_CLASS_JOYSTICK: u32 = 0x0000_0010;
const TRANSPORT_WIFI: i32 = 1;

#[derive(Default)]
struct Pad {
    lx: f32,
    ly: f32,
    rx: f32,
    ry: f32,
    arm_toggle: bool,
    emergency: bool,
    takeoff: bool,
    land: bool,
    flip: bool,
    calibrate: bool,
    last_key: u32,
    raw: [f32; 8],
}

pub fn run(app: AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("skyraptor"),
    );
    log::info!("android_main (native-res render)");

    let mut quit = false;
    let mut window: Option<NativeWindow> = None;
    let mut win_w = 0usize;
    let mut win_h = 0usize;
    let mut fb: Vec<u32> = Vec::new(); // native-resolution framebuffer
    let mut vid = vec![0u32; VIDEO_W * VIDEO_H]; // decoded video (source res)
    let mut link: Option<DroneLink> = None;
    let mut last_frame: Option<Instant> = None;
    let mut last_attempt: Option<Instant> = None;
    let mut pad = Pad::default();
    let mut armed = false;
    let mut prev_throttle = CENTER;
    let (mut fps_count, mut shown_fps) = (0u32, 0u32);
    let mut fps_since = Instant::now();
    let mut frame: u64 = 0;

    while !quit {
        let mut got_window = false;
        let mut lost_window = false;
        app.poll_events(Some(Duration::from_millis(16)), |event| match event {
            PollEvent::Main(MainEvent::InitWindow { .. }) => got_window = true,
            PollEvent::Main(MainEvent::TerminateWindow { .. }) => lost_window = true,
            PollEvent::Main(MainEvent::Destroy) => quit = true,
            _ => {}
        });
        // Re-acquire on (re)create; drop on terminate. Poll for the window each
        // frame rather than relying solely on the InitWindow event (which may not
        // fire on a warm resume).
        if got_window || lost_window {
            window = None;
            win_w = 0;
        }
        if window.is_none() {
            window = app.native_window();
        }
        if let Some(nw) = &window {
            if win_w == 0 {
                // Keep the window's native size; just force RGBA. Blit 1:1 -> crisp.
                let _ = nw.set_buffers_geometry(0, 0, Some(HardwareBufferFormat::R8G8B8A8_UNORM));
                win_w = nw.width().max(1) as usize;
                win_h = nw.height().max(1) as usize;
                fb = vec![0u32; win_w * win_h];
                log::info!("window {win_w}x{win_h}");
            }
        }
        frame += 1;

        // Connection watchdog: (re)bind to wifi + (re)start the link when no video
        // is arriving. NEVER while armed — don't disrupt a flight.
        let stale = last_frame.map_or(true, |t| t.elapsed() > Duration::from_secs(3));
        let ready = last_attempt.map_or(true, |t| t.elapsed() > Duration::from_secs(2));
        if !armed && stale && ready {
            last_attempt = Some(Instant::now());
            if bind_to_wifi() {
                if let Some(old) = link.take() {
                    old.stop();
                }
                match DroneLink::start(LinkConfig::default()) {
                    Ok(l) => {
                        log::info!("DroneLink (re)started");
                        link = Some(l);
                    }
                    Err(e) => log::error!("DroneLink failed: {e}"),
                }
            }
        }

        if let Ok(mut iter) = app.input_events_iter() {
            while iter.next(|e| handle_input(e, &mut pad)) {}
        }

        let dz = |v: f32| if v.abs() > DEADZONE { v } else { 0.0 };
        let shape = |raw: f32| axis_to_byte(expo(raw, EXPO) * MAX_DEFLECTION);
        let roll = shape(dz(pad.rx));
        let pitch = shape(dz(-pad.ry));
        let yaw = shape(dz(pad.lx));
        prev_throttle = ramp_toward(prev_throttle, shape(dz(-pad.ly)), THROTTLE_RAMP);
        let throttle = prev_throttle;
        let mut flags = 0u8;
        if pad.takeoff { flags |= FLAG_TAKEOFF; }
        if pad.land { flags |= FLAG_LAND; }
        if pad.flip { flags |= FLAG_FLIP; }
        if pad.calibrate { flags |= FLAG_CALIBRATE; }
        if pad.emergency { flags |= FLAG_EMERGENCY; armed = false; }

        if let Some(l) = &link {
            if pad.arm_toggle {
                armed = !armed;
                if armed { l.control.arm(); prev_throttle = CENTER; } else { l.control.disarm(); }
            }
            if pad.emergency { l.control.arm(); }
            l.control.set(ControlState { roll, pitch, throttle, yaw, flags });
            if !armed && !pad.emergency { l.control.disarm(); }
            let mut latest = None;
            while let Ok(f) = l.frames.try_recv() {
                latest = Some(f);
            }
            if let Some(jpeg) = latest {
                if decode_into(&jpeg, &mut vid) {
                    last_frame = Some(Instant::now());
                    fps_count += 1;
                }
            }
        }
        pad.arm_toggle = false;

        if fps_since.elapsed() >= Duration::from_secs(1) {
            shown_fps = fps_count;
            fps_count = 0;
            fps_since = Instant::now();
        }
        if frame % 60 == 0 {
            log::info!("[axes] X{:+.2} Y{:+.2} Z{:+.2} Rz{:+.2}", pad.raw[0], pad.raw[1], pad.raw[2], pad.raw[3]);
        }

        // --- render at native resolution ---
        if win_w > 0 && !fb.is_empty() {
            for px in fb.iter_mut() {
                *px = 0x0000_0000;
            }
            scale_video(&mut fb, win_w, win_h, &vid);
            let connected = last_frame.map_or(false, |t| t.elapsed() < Duration::from_secs(2));
            draw_hud(&mut fb, win_w, win_h, armed, connected, shown_fps, throttle, yaw, roll, pitch, flags, pad.last_key);
            if let Some(nw) = &window {
                blit(nw, &fb, win_w, win_h);
            }
        }
    }
    if let Some(l) = link {
        l.stop();
    }
    log::info!("android_main exiting");
}

/// Scale the 240x320 video into `fb` (native size), aspect-correct and centered.
fn scale_video(fb: &mut [u32], w: usize, h: usize, vid: &[u32]) {
    let scale = (w as f32 / VIDEO_W as f32).min(h as f32 / VIDEO_H as f32);
    let dw = (VIDEO_W as f32 * scale) as usize;
    let dh = (VIDEO_H as f32 * scale) as usize;
    if dw == 0 || dh == 0 {
        return;
    }
    let ox = (w - dw) / 2;
    let oy = (h - dh) / 2;
    for dy in 0..dh {
        let sy = dy * VIDEO_H / dh;
        let row = (oy + dy) * w + ox;
        let srow = sy * VIDEO_W;
        for dx in 0..dw {
            fb[row + dx] = vid[srow + dx * VIDEO_W / dw];
        }
    }
}

fn bind_to_wifi() -> bool {
    let ctx = ndk_context::android_context();
    let vm = match unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) } {
        Ok(v) => v,
        Err(_) => return false,
    };
    let mut env = match vm.attach_current_thread() {
        Ok(e) => e,
        Err(_) => return false,
    };
    let activity = unsafe { JObject::from_raw(ctx.context().cast()) };

    let mut try_bind = || -> jni::errors::Result<bool> {
        let name = env.new_string("connectivity")?;
        let cm = env
            .call_method(&activity, "getSystemService", "(Ljava/lang/String;)Ljava/lang/Object;", &[(&name).into()])?
            .l()?;
        let networks: JObjectArray = env
            .call_method(&cm, "getAllNetworks", "()[Landroid/net/Network;", &[])?
            .l()?
            .into();
        let len = env.get_array_length(&networks)?;
        for i in 0..len {
            let net = env.get_object_array_element(&networks, i)?;
            let caps = env
                .call_method(&cm, "getNetworkCapabilities", "(Landroid/net/Network;)Landroid/net/NetworkCapabilities;", &[(&net).into()])?
                .l()?;
            if caps.is_null() {
                continue;
            }
            let is_wifi = env
                .call_method(&caps, "hasTransport", "(I)Z", &[JValue::Int(TRANSPORT_WIFI)])?
                .z()?;
            if is_wifi {
                let ok = env
                    .call_method(&cm, "bindProcessToNetwork", "(Landroid/net/Network;)Z", &[(&net).into()])?
                    .z()?;
                return Ok(ok);
            }
        }
        Ok(false)
    };
    match try_bind() {
        Ok(b) => {
            log::info!("bind_to_wifi -> {b}");
            b
        }
        Err(e) => {
            log::error!("bind_to_wifi error: {e:?}");
            false
        }
    }
}

fn handle_input(event: &InputEvent, pad: &mut Pad) -> InputStatus {
    match event {
        InputEvent::MotionEvent(m) => {
            if u32::from(m.source()) & SOURCE_CLASS_JOYSTICK != 0 && m.pointer_count() > 0 {
                let p = m.pointer_at_index(0);
                pad.raw = [
                    p.axis_value(Axis::X),
                    p.axis_value(Axis::Y),
                    p.axis_value(Axis::Z),
                    p.axis_value(Axis::Rz),
                    p.axis_value(Axis::Rx),
                    p.axis_value(Axis::Ry),
                    p.axis_value(Axis::HatX),
                    p.axis_value(Axis::HatY),
                ];
                pad.lx = pad.raw[0];
                pad.ly = pad.raw[1];
                pad.rx = pad.raw[2];
                pad.ry = pad.raw[3];
                return InputStatus::Handled;
            }
            InputStatus::Unhandled
        }
        InputEvent::KeyEvent(k) => {
            let down = matches!(k.action(), KeyAction::Down);
            let kc = k.key_code();
            if down {
                pad.last_key = u32::from(kc);
            }
            match kc {
                Keycode::ButtonStart => {
                    if down && k.repeat_count() == 0 {
                        pad.arm_toggle = true;
                    }
                }
                Keycode::ButtonSelect | Keycode::ButtonMode => pad.emergency = down,
                Keycode::ButtonA => pad.land = down,
                Keycode::ButtonB => pad.takeoff = down,
                Keycode::ButtonY => pad.flip = down,
                Keycode::ButtonX => pad.calibrate = down,
                _ => {}
            }
            InputStatus::Handled
        }
        _ => InputStatus::Unhandled,
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_hud(
    fb: &mut [u32],
    w: usize,
    h: usize,
    armed: bool,
    connected: bool,
    fps: u32,
    throttle: u8,
    yaw: u8,
    roll: u8,
    pitch: u8,
    flags: u8,
    last_key: u32,
) {
    let mut c = hud::Canvas { buf: fb, w, h };
    let s = (w / 360).max(2); // font scale: crisp at native res
    let g = 8 * s; // glyph cell
    // Safe-area inset: clears rounded corners + top camera cutout.
    let mx = w / 18;
    let top = h / 12;
    let bot = h / 18;
    let (x0, y0, x1, y1) = (mx, top, w - mx, h - bot);
    let fcol = if armed { hud::GREEN } else { hud::RED };

    // inset neon border
    c.hline(x0, y0, x1 - x0, fcol);
    c.hline(x0, y1, x1 - x0, fcol);
    c.vline(x0, y0, y1 - y0, fcol);
    c.vline(x1, y0, y1 - y0, fcol);

    // top status panel
    let ph = g * 2 + 6 * s;
    c.panel(x0 + s, y0 + s, x1 - x0 - 2 * s, ph, 170);
    let tx = x0 + 2 * s;
    let (txt, col) = if armed { ("[ARMED]", hud::GREEN) } else { ("[STANDBY]", hud::AMBER) };
    c.glow_text(tx, y0 + 2 * s, txt, col, s);
    let link = if connected { "LINK" } else { "NO SIG" };
    c.glow_text(x1 - 12 * g, y0 + 2 * s, link, if connected { hud::CYAN } else { hud::AMBER }, s);
    c.glow_text(x1 - 5 * g, y0 + 2 * s, &format!("FPS{fps:02}"), hud::CYAN, s);
    // throttle bar row
    c.glow_text(tx, y0 + 2 * s + g, "THR", hud::CYAN, s);
    let bar_x = tx + 4 * g;
    c.bar(bar_x, y0 + 2 * s + g, x1 - bar_x - 2 * s, g - 2 * s, throttle as f32 / 255.0, if armed { hud::GREEN } else { hud::AMBER });

    // stick boxes near the bottom safe corners
    let bs = w / 4;
    c.stick_box(x0 + 2 * s, y1 - bs - g, bs, yaw, throttle, hud::MAGENTA);
    c.glow_text(x0 + 2 * s, y1 - g + s, "YAW/THR", hud::MAGENTA, s.max(2));
    c.stick_box(x1 - bs - 2 * s, y1 - bs - g, bs, roll, pitch, hud::CYAN);
    c.glow_text(x1 - 8 * g, y1 - g + s, "ROL/PIT", hud::CYAN, s.max(2));

    // small debug line (temporary) above the stick boxes
    c.glow_text(tx, y1 - bs - g - g, &format!("FLG{flags:02X} K{last_key}"), hud::GREEN, s.max(2));
}

fn decode_into(jpeg: &[u8], vid: &mut [u32]) -> bool {
    let Ok(rgb) = JpegDecoder::new(jpeg).decode() else { return false };
    if rgb.len() < VIDEO_W * VIDEO_H * 3 {
        return false;
    }
    for (px, chunk) in vid.iter_mut().zip(rgb.chunks_exact(3)) {
        *px = ((chunk[0] as u32) << 16) | ((chunk[1] as u32) << 8) | chunk[2] as u32;
    }
    true
}

fn blit(nw: &NativeWindow, fb: &[u32], w_buf: usize, h_buf: usize) {
    let Ok(mut guard) = nw.lock(None) else { return };
    let (w, h, stride) = (guard.width().min(w_buf), guard.height().min(h_buf), guard.stride());
    let Some(bytes) = guard.bytes() else { return };
    for y in 0..h {
        for x in 0..w {
            let px = fb[y * w_buf + x];
            let i = (y * stride + x) * 4;
            bytes[i] = MaybeUninit::new((px >> 16) as u8);
            bytes[i + 1] = MaybeUninit::new((px >> 8) as u8);
            bytes[i + 2] = MaybeUninit::new(px as u8);
            bytes[i + 3] = MaybeUninit::new(0xFF);
        }
    }
}
