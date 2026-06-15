//! Android app loop: input → control, video → screen. Android-only.
//!
//! Renders at the phone's NATIVE resolution: the video is decoded at 240x320 and
//! scaled (aspect-correct, centered) into the framebuffer, and the HUD is drawn
//! crisp at full res inside a safe-area inset (clears rounded corners + camera
//! cutout). Reuses `protocol`/`net`/`hud`. Stays disarmed until armed via Start.

use std::mem::MaybeUninit;
use std::time::{Duration, Instant};

use android_activity::input::{Axis, InputEvent, KeyAction, Keycode};
use android_activity::{AndroidApp, InputStatus, MainEvent, PollEvent, WindowManagerFlags};
use jni::objects::{JObject, JObjectArray, JValue};
use ndk::{hardware_buffer_format::HardwareBufferFormat, native_window::NativeWindow};
use net::{DroneLink, LinkConfig};
use protocol::{
    apply_trim, axis_to_byte, expo, ramp_toward, ControlState, CENTER, FLAG_CALIBRATE,
    FLAG_EMERGENCY, FLAG_FLIP, FLAG_HEADLESS, FLAG_LAND, FLAG_TAKEOFF,
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
/// Drone AP SSID prefix (observed: "WIFI_8K__<mac>"), matched as a prefix pattern.
const SSID_PREFIX: &str = "WIFI_8K__";

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
    hx: f32, // D-pad hat X (-1 left / +1 right)
    hy: f32, // D-pad hat Y (-1 up / +1 down)
    headless_toggle: bool,
    trim_reset: bool,
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
    let mut wifi_requested = false; // one-shot WifiNetworkSpecifier request
    let mut perm_requested = false; // one-shot NEARBY_WIFI_DEVICES request
    let mut pad = Pad::default();
    let mut armed = false;
    let mut prev_throttle = CENTER;
    let (mut trim_roll, mut trim_pitch): (i8, i8) = (0, 0);
    let mut headless = false;
    let (mut prev_hx, mut prev_hy) = (0.0f32, 0.0f32);
    let (mut fps_count, mut shown_fps) = (0u32, 0u32);
    let mut fps_since = Instant::now();
    let mut frame: u64 = 0;
    let app_start = Instant::now();

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
                // Keep the screen awake (thread-safe native window flag) so the
                // phone never sleeps mid-flight and drops input focus / freezes us.
                app.set_window_flags(WindowManagerFlags::KEEP_SCREEN_ON, WindowManagerFlags::empty());
            }
        }
        frame += 1;

        // Connection watchdog: (re)bind to wifi + (re)start the link when no video
        // is arriving. NEVER while armed — don't disrupt a flight.
        let stale = last_frame.map_or(true, |t| t.elapsed() > Duration::from_secs(3));
        let ready = last_attempt.map_or(true, |t| t.elapsed() > Duration::from_secs(2));
        if !armed && stale && ready {
            last_attempt = Some(Instant::now());
            // First time we're disconnected, ask the system to join the drone AP.
            // The WifiNetworkSpecifier scan needs NEARBY_WIFI_DEVICES; we can't
            // request it in-app (ndk_context gives the Application, not the Activity,
            // and runtime-permission UI needs the Activity/UI thread), so it must be
            // granted once in Settings. (If already on the drone wifi manually we're
            // not stale, so this never fires.)
            if nearby_wifi_granted() {
                // Grace period: if we're already on the drone wifi, video arrives
                // within a couple seconds (not stale) and we never prompt.
                if !wifi_requested && app_start.elapsed() > Duration::from_secs(4) {
                    wifi_requested = true;
                    request_drone_wifi();
                }
            } else if !perm_requested {
                perm_requested = true;
                log::warn!("NEARBY_WIFI_DEVICES not granted — enable it in Settings to auto-join the drone wifi");
            }
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

        // D-pad trim (edge-triggered ±1 per press), L1 headless toggle, R1 reset trim.
        const TRIM_LIMIT: i8 = 40;
        if pad.hx > 0.5 && prev_hx <= 0.5 { trim_roll = (trim_roll + 1).min(TRIM_LIMIT); }
        if pad.hx < -0.5 && prev_hx >= -0.5 { trim_roll = (trim_roll - 1).max(-TRIM_LIMIT); }
        if pad.hy < -0.5 && prev_hy >= -0.5 { trim_pitch = (trim_pitch + 1).min(TRIM_LIMIT); }
        if pad.hy > 0.5 && prev_hy <= 0.5 { trim_pitch = (trim_pitch - 1).max(-TRIM_LIMIT); }
        prev_hx = pad.hx;
        prev_hy = pad.hy;
        if pad.headless_toggle {
            headless = !headless;
            pad.headless_toggle = false;
        }
        if pad.trim_reset {
            trim_roll = 0;
            trim_pitch = 0;
            pad.trim_reset = false;
        }

        let dz = |v: f32| if v.abs() > DEADZONE { v } else { 0.0 };
        let shape = |raw: f32| axis_to_byte(expo(raw, EXPO) * MAX_DEFLECTION);
        let roll = apply_trim(shape(dz(pad.rx)), trim_roll);
        let pitch = apply_trim(shape(dz(-pad.ry)), trim_pitch);
        let yaw = shape(dz(pad.lx));
        prev_throttle = ramp_toward(prev_throttle, shape(dz(-pad.ly)), THROTTLE_RAMP);
        let throttle = prev_throttle;
        let mut flags = 0u8;
        if pad.takeoff { flags |= FLAG_TAKEOFF; }
        if pad.land { flags |= FLAG_LAND; }
        if pad.flip { flags |= FLAG_FLIP; }
        if pad.calibrate { flags |= FLAG_CALIBRATE; }
        if headless { flags |= FLAG_HEADLESS; }
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
        // --- render at native resolution ---
        if win_w > 0 && !fb.is_empty() {
            for px in fb.iter_mut() {
                *px = 0x0000_0000;
            }
            scale_video(&mut fb, win_w, win_h, &vid);
            let connected = last_frame.map_or(false, |t| t.elapsed() < Duration::from_secs(2));
            draw_hud(&mut fb, win_w, win_h, armed, connected, shown_fps, throttle, yaw, roll, pitch, frame, trim_roll, trim_pitch, headless);
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

    let res = (|| -> jni::errors::Result<bool> {
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
    })();
    let _ = env.exception_clear();
    match res {
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

/// Ask Android to connect to the drone's WiFi AP (SSID prefix `WIFI_8K__`) via
/// `ConnectivityManager.requestNetwork(WifiNetworkSpecifier, PendingIntent)`. Shows
/// a one-time system approval dialog; the request persists so the OS keeps the
/// drone connected while the app runs. Pure JNI (PendingIntent overload avoids the
/// NetworkCallback subclass that can't be made from Rust).
fn request_drone_wifi() -> bool {
    const PATTERN_PREFIX: i32 = 1; // android.os.PatternMatcher.PATTERN_PREFIX
    const FLAG_IMMUTABLE: i32 = 0x0400_0000;
    const FLAG_UPDATE_CURRENT: i32 = 0x0800_0000;

    let ctx = ndk_context::android_context();
    let Ok(vm) = (unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) }) else { return false };
    let Ok(mut env) = vm.attach_current_thread() else { return false };
    let activity = unsafe { JObject::from_raw(ctx.context().cast()) };

    let res = (|| -> jni::errors::Result<()> {
        let name = env.new_string("connectivity")?;
        let cm = env
            .call_method(&activity, "getSystemService", "(Ljava/lang/String;)Ljava/lang/Object;", &[(&name).into()])?
            .l()?;

        // PatternMatcher(SSID_PREFIX, PATTERN_PREFIX)
        let prefix = env.new_string(SSID_PREFIX)?;
        let pm = env.new_object(
            "android/os/PatternMatcher",
            "(Ljava/lang/String;I)V",
            &[(&prefix).into(), JValue::Int(PATTERN_PREFIX)],
        )?;

        // WifiNetworkSpecifier.Builder().setSsidPattern(pm).build()
        let b = env.new_object("android/net/wifi/WifiNetworkSpecifier$Builder", "()V", &[])?;
        let b = env
            .call_method(&b, "setSsidPattern", "(Landroid/os/PatternMatcher;)Landroid/net/wifi/WifiNetworkSpecifier$Builder;", &[(&pm).into()])?
            .l()?;
        let specifier = env
            .call_method(&b, "build", "()Landroid/net/wifi/WifiNetworkSpecifier;", &[])?
            .l()?;

        // NetworkRequest.Builder().addTransportType(WIFI).setNetworkSpecifier(spec).build()
        let rb = env.new_object("android/net/NetworkRequest$Builder", "()V", &[])?;
        let rb = env
            .call_method(&rb, "addTransportType", "(I)Landroid/net/NetworkRequest$Builder;", &[JValue::Int(TRANSPORT_WIFI)])?
            .l()?;
        let rb = env
            .call_method(&rb, "setNetworkSpecifier", "(Landroid/net/NetworkSpecifier;)Landroid/net/NetworkRequest$Builder;", &[(&specifier).into()])?
            .l()?;
        let request = env.call_method(&rb, "build", "()Landroid/net/NetworkRequest;", &[])?.l()?;

        // PendingIntent.getBroadcast(activity, 0, new Intent(action), IMMUTABLE|UPDATE_CURRENT)
        let action = env.new_string("app.skyraptor.drcx5.WIFI")?;
        let intent = env.new_object("android/content/Intent", "(Ljava/lang/String;)V", &[(&action).into()])?;
        let pending = env
            .call_static_method(
                "android/app/PendingIntent",
                "getBroadcast",
                "(Landroid/content/Context;ILandroid/content/Intent;I)Landroid/app/PendingIntent;",
                &[(&activity).into(), JValue::Int(0), (&intent).into(), JValue::Int(FLAG_IMMUTABLE | FLAG_UPDATE_CURRENT)],
            )?
            .l()?;

        // cm.requestNetwork(request, pending)
        env.call_method(
            &cm,
            "requestNetwork",
            "(Landroid/net/NetworkRequest;Landroid/app/PendingIntent;)V",
            &[(&request).into(), (&pending).into()],
        )?;
        Ok(())
    })();
    let _ = env.exception_clear();
    match res {
        Ok(_) => {
            log::info!("requestNetwork({SSID_PREFIX}*) issued");
            true
        }
        Err(e) => {
            log::error!("request_drone_wifi error: {e:?}");
            false
        }
    }
}

/// Is NEARBY_WIFI_DEVICES granted? (Needed for the WifiNetworkSpecifier scan.)
fn nearby_wifi_granted() -> bool {
    let ctx = ndk_context::android_context();
    let Ok(vm) = (unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) }) else { return false };
    let Ok(mut env) = vm.attach_current_thread() else { return false };
    let activity = unsafe { JObject::from_raw(ctx.context().cast()) };
    let r = (|| -> jni::errors::Result<i32> {
        let perm = env.new_string("android.permission.NEARBY_WIFI_DEVICES")?;
        env.call_method(&activity, "checkSelfPermission", "(Ljava/lang/String;)I", &[(&perm).into()])?
            .i()
    })();
    let _ = env.exception_clear();
    matches!(r, Ok(0)) // PackageManager.PERMISSION_GRANTED
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
                pad.hx = p.axis_value(Axis::HatX);
                pad.hy = p.axis_value(Axis::HatY);
                return InputStatus::Handled;
            }
            InputStatus::Unhandled
        }
        InputEvent::KeyEvent(k) => {
            let down = matches!(k.action(), KeyAction::Down);
            let kc = k.key_code();
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
                Keycode::ButtonL1 => {
                    if down && k.repeat_count() == 0 {
                        pad.headless_toggle = true;
                    }
                }
                Keycode::ButtonR1 => {
                    if down && k.repeat_count() == 0 {
                        pad.trim_reset = true;
                    }
                }
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
    frame: u64,
    trim_roll: i8,
    trim_pitch: i8,
    headless: bool,
) {
    let mut c = hud::Canvas { buf: fb, w, h };
    let s = (w / 360).max(2); // font scale: crisp at native res
    let g = 8 * s; // glyph cell
    // Safe-area inset: clears rounded corners + top camera cutout.
    let mx = w / 18;
    let top = h / 12;
    let bot = h / 18;
    let (x0, y0, x1, y1) = (mx, top, w - mx, h - bot);

    // top status panel
    let ph = g * 3 + 6 * s;
    c.panel(x0 + s, y0 + s, x1 - x0 - 2 * s, ph, 170);
    let tx = x0 + 2 * s;
    let row0 = y0 + 2 * s;
    let (txt, col) = if armed { ("[ARMED]", hud::GREEN) } else { ("[STANDBY]", hud::AMBER) };
    c.glow_text(tx, row0, txt, col, s);
    let lk = if connected { "LINK" } else { "NOSIG" };
    c.glow_text(x1 - 13 * g, row0, &format!("{lk} {fps:02}"), if connected { hud::CYAN } else { hud::AMBER }, s);
    // link-quality meter from video fps (proxy for usable link; no perms needed):
    // bars drop as you approach range limits — fly back before it hits zero.
    let bars: u32 = if fps >= 18 { 3 } else if fps >= 10 { 2 } else if fps >= 3 { 1 } else { 0 };
    let qc = if bars >= 3 { hud::GREEN } else if bars == 2 { hud::AMBER } else { hud::RED };
    let bx = x1 - 4 * g;
    for i in 0..3u32 {
        let bh = (i as usize + 1) * 2 * s;
        c.fill(bx + i as usize * 3 * s, row0 + g - bh, 2 * s, bh, if i < bars { qc } else { 0x0033_3333 });
    }
    // throttle bar row
    c.glow_text(tx, y0 + 2 * s + g, "THR", hud::CYAN, s);
    let bar_x = tx + 4 * g;
    c.bar(bar_x, y0 + 2 * s + g, x1 - bar_x - 2 * s, g - 2 * s, throttle as f32 / 255.0, if armed { hud::GREEN } else { hud::AMBER });

    // trim + headless row
    let row2 = y0 + 2 * s + 2 * g;
    c.glow_text(tx, row2, &format!("TRM R{trim_roll:+03} P{trim_pitch:+03}"), hud::CYAN, s);
    if headless {
        c.glow_text(x1 - 9 * g, row2, "HEADLESS", hud::MAGENTA, s);
    }

    // stick boxes near the bottom safe corners
    let bs = w / 4;
    c.stick_box(x0 + 2 * s, y1 - bs - g, bs, yaw, throttle, hud::MAGENTA);
    c.glow_text(x0 + 2 * s, y1 - g + s, "YAW/THR", hud::MAGENTA, s.max(2));
    c.stick_box(x1 - bs - 2 * s, y1 - bs - g, bs, roll, pitch, hud::CYAN);
    c.glow_text(x1 - 8 * g, y1 - g + s, "ROL/PIT", hud::CYAN, s.max(2));

    // Loud, blinking "LINK LOST" banner when no video is arriving — fly back!
    if !connected && (frame / 18).is_multiple_of(2) {
        let bscale = s + 1;
        let msg = "LINK LOST";
        let tw = msg.len() * 8 * bscale;
        let bx = w.saturating_sub(tw) / 2;
        let by = h * 2 / 5;
        c.panel(bx.saturating_sub(6), by.saturating_sub(6), tw + 12, 8 * bscale + 12, 210);
        c.glow_text(bx, by, msg, hud::RED, bscale);
    }
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
