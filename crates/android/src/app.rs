//! Android app loop: input → control, video → screen. Android-only.
//!
//! Renders at the phone's NATIVE resolution: the video is decoded at 240x320 and
//! scaled (aspect-correct, centered) into the framebuffer, and the HUD is drawn
//! crisp at full res inside a safe-area inset (clears rounded corners + camera
//! cutout). Reuses `protocol`/`net`/`hud`. Stays disarmed until armed via Start.

use std::mem::MaybeUninit;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use android_activity::input::{Axis, InputEvent, KeyAction, MotionAction};
use android_activity::{AndroidApp, InputStatus, MainEvent, PollEvent, WindowManagerFlags};
use jni::objects::{JObject, JObjectArray, JValue};
use ndk::{hardware_buffer_format::HardwareBufferFormat, native_window::NativeWindow};
use net::{DroneLink, LinkConfig};
use protocol::{
    apply_trim, avi::AviWriter, axis_to_byte, expo, ramp_toward, ControlState, CENTER,
    FLAG_CALIBRATE, FLAG_EMERGENCY, FLAG_FLIP, FLAG_HEADLESS, FLAG_LAND, FLAG_TAKEOFF,
};
use zune_jpeg::JpegDecoder;

use crate::settings;

const VIDEO_W: usize = 240;
const VIDEO_H: usize = 320;
const EXPO: f32 = 0.25;
const THROTTLE_RAMP: u8 = 6;
const DEADZONE: f32 = 0.06;
const SOURCE_CLASS_JOYSTICK: u32 = 0x0000_0010;
const SOURCE_CLASS_POINTER: u32 = 0x0000_0002; // touchscreen
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
    estop: bool, // emergency one-shot edge (for the sim kill)
    speed_cycle: bool, // one-shot edge: cycle the speed preset
    hx: f32, // D-pad hat X (-1 left / +1 right) — roll trim
    hy: f32, // D-pad hat Y (-1 up / +1 down) — pitch trim
    takeoff: bool,
    land: bool,
    flip: bool,
    calibrate: bool,
    rtrig: f32, // right trigger (analog 0..1)
    ltrig: f32, // left trigger (analog 0..1)
    r2: bool, // right trigger as a digital button
    l2: bool, // left trigger as a digital button
    headless_toggle: bool,
    trim_reset: bool,
    tap: Option<(f32, f32)>, // last touch-down screen coords
    captured_key: Option<u32>, // last gamepad keycode down (for remapping)
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
    let (mut trim_roll, mut trim_pitch): (i8, i8) = (0, 0);
    let mut headless = false;
    let (mut prev_hx, mut prev_hy) = (0.0f32, 0.0f32);
    let (mut fps_count, mut shown_fps) = (0u32, 0u32);
    let mut fps_since = Instant::now();
    let mut frame: u64 = 0;
    let cfg_path = files_dir().map(|d| format!("{d}/bindings.txt"));
    let mut bindings = cfg_path.as_deref().map(settings::load).unwrap_or_default();
    // Photos + videos: saved to the app's external files dir (no permission needed,
    // pullable via adb / visible in a file manager).
    let media_dir = external_files_dir();
    let snap_dir = media_dir.as_ref().map(|d| format!("{d}/snapshots"));
    let video_dir = media_dir.as_ref().map(|d| format!("{d}/videos"));
    let mut last_jpeg: Option<Vec<u8>> = None; // latest decoded frame's raw JPEG
    let mut snap_note: Option<(String, Instant)> = None; // capture confirmation
    let mut snap_count: u32 = 0;
    let mut recorder: Option<AviWriter<std::fs::File>> = None; // active MJPEG recording
    let mut rec_start: Option<Instant> = None;
    let mut settings_open = false;
    let mut listening: Option<settings::Action> = None;
    let mut preview_open = false;
    let mut heading = 0.0f32; // virtual-drone yaw (sim)
    let mut sim_alt = 0.0f32; // sim altitude 0..~1.2
    let mut sim_alt_target: Option<f32> = None; // active takeoff/land glide target
    let mut sim_spin = 0.0f32; // rotor spin phase (radians)
    let mut sim_killed = false; // emergency motor cut active
    let mut sim_flip = 0.0f32; // flip progress 1->0 while animating
    let mut sim_prev_flip = false;
    let (mut sim_flip_rc, mut sim_flip_pc) = (1.0f32, 0.0f32); // flip axis: roll/pitch components
    let mut sim_prev_takeoff = false;
    let mut sim_prev_land = false;

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

        // Connection watchdog: silently (re)bind to wifi + (re)start the link when
        // no video is arriving. NEVER while armed — don't disrupt a flight. This does
        // NOT pop the system wifi-join dialog: it only recovers when we're already on
        // the drone AP. Joining a lost/absent AP is done solely by tapping RECONNECT,
        // so shutting the drone off (e.g. to use the sim) never prompts.
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
            while iter.next(|e| handle_input(e, &mut pad, &bindings)) {}
        }

        // Receive video always, so the feed stays live behind the settings overlay.
        if let Some(l) = &link {
            let mut latest = None;
            while let Ok(f) = l.frames.try_recv() {
                latest = Some(f);
            }
            if let Some(jpeg) = latest {
                if decode_into(&jpeg, &mut vid) {
                    last_frame = Some(Instant::now());
                    fps_count += 1;
                    if let Some(rec) = &mut recorder {
                        let _ = rec.write_frame(&jpeg); // mux this frame into the AVI
                    }
                    last_jpeg = Some(jpeg); // keep raw bytes for snapshot (no re-encode)
                }
            }
        }
        let connected = last_frame.map_or(false, |t| t.elapsed() < Duration::from_secs(2));

        // Speed preset: a bound button cycles LOW -> MED -> HIGH (in flight/sim, not
        // while remapping in settings). Scales max stick deflection. Persisted.
        if pad.speed_cycle {
            pad.speed_cycle = false;
            if !settings_open {
                bindings.speed = (bindings.speed + 1) % 3;
                if let Some(p) = &cfg_path {
                    settings::save(p, &bindings);
                }
            }
        }
        let max_defl = settings::SPEED_DEFLECTION[(bindings.speed as usize).min(2)];
        let dz = |v: f32| if v.abs() > DEADZONE { v } else { 0.0 };
        let shape = |raw: f32| axis_to_byte(expo(raw, EXPO) * max_defl);
        let (mut roll, mut pitch, mut yaw, mut throttle, mut flags) = (CENTER, CENTER, CENTER, CENTER, 0u8);

        if settings_open {
            // Stay disarmed; never send active control while remapping.
            armed = false;
            if let Some(l) = &link {
                l.control.disarm();
            }
            if let Some(a) = listening {
                if let Some(kc) = pad.captured_key.take() {
                    bindings.set(a, kc);
                    listening = None;
                    if let Some(p) = &cfg_path {
                        settings::save(p, &bindings);
                    }
                }
            }
            if let Some((tx_, ty_)) = pad.tap.take() {
                match settings_hit(tx_ as usize, ty_ as usize, win_w, win_h) {
                    Some(SettingsHit::Row(a)) => listening = Some(a),
                    Some(SettingsHit::ThrottleToggle) => {
                        bindings.throttle_triggers = !bindings.throttle_triggers;
                        if let Some(p) = &cfg_path {
                            settings::save(p, &bindings);
                        }
                    }
                    Some(SettingsHit::Reset) => {
                        bindings = settings::Bindings::default();
                        if let Some(p) = &cfg_path {
                            settings::save(p, &bindings);
                        }
                    }
                    Some(SettingsHit::Done) => {
                        settings_open = false;
                        listening = None;
                    }
                    None => {}
                }
            }
            pad.arm_toggle = false; // discard control one-shots while in settings
            pad.estop = false;
        } else {
            // D-pad trims roll (left/right) and pitch (up/down), one step per press.
            const TRIM_LIMIT: i8 = 40;
            if pad.hx > 0.5 && prev_hx <= 0.5 { trim_roll = (trim_roll + 1).min(TRIM_LIMIT); }
            if pad.hx < -0.5 && prev_hx >= -0.5 { trim_roll = (trim_roll - 1).max(-TRIM_LIMIT); }
            if pad.hy < -0.5 && prev_hy >= -0.5 { trim_pitch = (trim_pitch + 1).min(TRIM_LIMIT); }
            if pad.hy > 0.5 && prev_hy <= 0.5 { trim_pitch = (trim_pitch - 1).max(-TRIM_LIMIT); }
            prev_hx = pad.hx;
            prev_hy = pad.hy;
            if pad.headless_toggle { headless = !headless; pad.headless_toggle = false; }
            if pad.trim_reset { trim_roll = 0; trim_pitch = 0; pad.trim_reset = false; }

            roll = apply_trim(shape(dz(pad.rx)), trim_roll);
            pitch = apply_trim(shape(dz(-pad.ry)), trim_pitch);
            yaw = shape(dz(pad.lx));
            // Throttle source: L2/R2 triggers (R2 up, L2 down) or the left-stick Y.
            let thr_in = if bindings.throttle_triggers {
                pad.rtrig.max(if pad.r2 { 1.0 } else { 0.0 }) - pad.ltrig.max(if pad.l2 { 1.0 } else { 0.0 })
            } else {
                -pad.ly
            };
            prev_throttle = ramp_toward(prev_throttle, shape(dz(thr_in)), THROTTLE_RAMP);
            throttle = prev_throttle;

            if pad.takeoff { flags |= FLAG_TAKEOFF; }
            if pad.land { flags |= FLAG_LAND; }
            if pad.flip { flags |= FLAG_FLIP; }
            if pad.calibrate { flags |= FLAG_CALIBRATE; }
            if headless { flags |= FLAG_HEADLESS; }
            if pad.emergency { flags |= FLAG_EMERGENCY; armed = false; }

            if preview_open {
                // Virtual-drone sim: never send control; integrate yaw into heading.
                armed = false;
                if let Some(l) = &link {
                    l.control.disarm();
                }
                // Emergency button: cut the motors. The drone drops fast and stays
                // grounded until the next takeoff re-arms it.
                if pad.estop {
                    sim_killed = true;
                    sim_alt_target = None;
                }
                // Takeoff (edge): if grounded, glide up to 25% altitude (clears a kill).
                // Land (edge): if airborne, glide down to the ground.
                if pad.takeoff && !sim_prev_takeoff && sim_alt <= 0.01 {
                    sim_alt_target = Some(0.25);
                    sim_killed = false;
                }
                sim_prev_takeoff = pad.takeoff;
                if pad.land && !sim_prev_land && sim_alt > 0.01 { sim_alt_target = Some(0.0); }
                sim_prev_land = pad.land;
                let thr_def = (throttle as f32 - 128.0) / 128.0;
                if sim_killed {
                    // Motors cut: free-fall to the ground, ignore sticks.
                    sim_alt = (sim_alt - 0.05).max(0.0);
                } else {
                    // Throttle fine-tunes altitude; using it cancels any glide.
                    if thr_def.abs() > 0.1 { sim_alt_target = None; }
                    // Ease toward an active takeoff/land target.
                    if let Some(t) = sim_alt_target {
                        let step = 0.012;
                        if (sim_alt - t).abs() <= step {
                            sim_alt = t;
                            sim_alt_target = None;
                        } else {
                            sim_alt += step.copysign(t - sim_alt);
                        }
                    }
                    sim_alt = (sim_alt + thr_def * 0.02).clamp(0.0, 1.2);
                }
                // Flip on the press edge, aimed by the right stick: push right/left
                // for a roll flip, forward/back for a pitch flip (centered = roll right).
                if pad.flip && !sim_prev_flip {
                    sim_flip = 1.0;
                    let (dx, dy) = (pad.rx, -pad.ry);
                    if dx.abs() < 0.25 && dy.abs() < 0.25 {
                        sim_flip_rc = 1.0;
                        sim_flip_pc = 0.0;
                    } else if dx.abs() >= dy.abs() {
                        sim_flip_rc = dx.signum();
                        sim_flip_pc = 0.0;
                    } else {
                        sim_flip_rc = 0.0;
                        sim_flip_pc = dy.signum();
                    }
                }
                sim_prev_flip = pad.flip;
                if sim_flip > 0.0 {
                    sim_flip = (sim_flip - 0.04).max(0.0);
                }
                // Rotors spin while "flying"; spin rate scales with throttle. Yaw,
                // roll and pitch only respond while the motors are spinning — a
                // grounded or killed drone holds its heading and stays level.
                let sim_motors = !sim_killed && (sim_alt > 0.02 || pad.takeoff || sim_flip > 0.0);
                if sim_motors {
                    sim_spin = (sim_spin + 0.5 + thr_def.max(0.0) * 0.8) % std::f32::consts::TAU;
                    // Yaw inverted in the sim view only (flight control unchanged).
                    heading -= (yaw as f32 - 128.0) / 128.0 * 0.10;
                }
                if let Some((tx_, ty_)) = pad.tap.take() {
                    let (px, py) = (tx_ as usize, ty_ as usize);
                    let (x, y, bw, bh) = exit_btn(win_w, win_h);
                    if px >= x && px < x + bw && py >= y && py < y + bh {
                        preview_open = false;
                    }
                }
                pad.arm_toggle = false;
                pad.headless_toggle = false;
                pad.trim_reset = false;
                pad.estop = false;
            } else {
                if let Some(l) = &link {
                    if pad.arm_toggle {
                        armed = !armed;
                        if armed { l.control.arm(); prev_throttle = CENTER; } else { l.control.disarm(); }
                    }
                    if pad.emergency { l.control.arm(); }
                    l.control.set(ControlState { roll, pitch, throttle, yaw, flags });
                    if !armed && !pad.emergency { l.control.disarm(); }
                }
                pad.arm_toggle = false;
                pad.estop = false;

                // Taps: KEY MAP / SIM (disarmed only) or reconnect.
                if let Some((tx_, ty_)) = pad.tap.take() {
                    let (px, py) = (tx_ as usize, ty_ as usize);
                    let inside = |(x, y, bw, bh): (usize, usize, usize, usize)| px >= x && px < x + bw && py >= y && py < y + bh;
                    if !armed && inside(menu_btn(win_w, win_h)) {
                        settings_open = true;
                        listening = None;
                    } else if !armed && inside(sim_btn(win_w, win_h)) {
                        preview_open = true;
                        sim_alt = 0.0;
                        sim_flip = 0.0;
                        heading = 0.0;
                    } else if connected && inside(photo_btn(win_w, win_h)) {
                        // Snapshot: write the latest raw JPEG straight to a file.
                        match (&snap_dir, &last_jpeg) {
                            (Some(dir), Some(j)) => match save_snapshot(dir, j) {
                                Ok(name) => {
                                    snap_count += 1;
                                    log::info!("snapshot saved: {name}");
                                    snap_note = Some((format!("PHOTO SAVED ({snap_count})"), Instant::now()));
                                }
                                Err(e) => {
                                    log::error!("snapshot failed: {e}");
                                    snap_note = Some(("PHOTO FAILED".into(), Instant::now()));
                                }
                            },
                            _ => snap_note = Some(("NO FRAME".into(), Instant::now())),
                        }
                    } else if connected && inside(rec_btn(win_w, win_h)) {
                        // Toggle MJPEG-in-AVI recording.
                        if let Some(rec) = recorder.take() {
                            let n = rec.frame_count();
                            let _ = rec.finish();
                            rec_start = None;
                            snap_note = Some((format!("VIDEO SAVED ({n}f)"), Instant::now()));
                        } else if let Some(dir) = &video_dir {
                            let ms = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis()).unwrap_or(0);
                            let started = std::fs::create_dir_all(dir)
                                .and_then(|_| std::fs::File::create(format!("{dir}/rec_{ms}.avi")))
                                .and_then(|f| AviWriter::new(f, VIDEO_W as u32, VIDEO_H as u32, 20));
                            match started {
                                Ok(w) => {
                                    recorder = Some(w);
                                    rec_start = Some(Instant::now());
                                    snap_note = Some(("REC STARTED".into(), Instant::now()));
                                }
                                Err(e) => {
                                    log::error!("record start failed: {e}");
                                    snap_note = Some(("REC FAILED".into(), Instant::now()));
                                }
                            }
                        }
                    } else if !connected {
                        // No video: tapping anywhere on the feed asks the system to
                        // join the drone AP. The WifiNetworkSpecifier scan needs
                        // NEARBY_WIFI_DEVICES, granted once in Settings.
                        if nearby_wifi_granted() {
                            request_drone_wifi();
                        } else {
                            log::warn!("NEARBY_WIFI_DEVICES not granted — enable it in Settings to join the drone wifi");
                        }
                        last_attempt = None; // let the watchdog rebind+restart promptly
                    }
                }
            }
        }
        pad.captured_key = None;

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
            if settings_open {
                draw_settings(&mut fb, win_w, win_h, &bindings, listening);
            } else if preview_open {
                let flip_angle = if sim_flip > 0.0 { (1.0 - sim_flip) * std::f32::consts::TAU } else { 0.0 };
                let (flip_roll, flip_pitch) = (flip_angle * sim_flip_rc, flip_angle * sim_flip_pc);
                draw_preview(&mut fb, win_w, win_h, roll, pitch, yaw, throttle, &pad, heading, sim_alt, flip_roll, flip_pitch, sim_spin, sim_killed);
            } else {
                let note = snap_note
                    .as_ref()
                    .filter(|(_, t)| t.elapsed() < Duration::from_millis(1500))
                    .map(|(m, _)| m.as_str());
                let rec_secs = rec_start.map(|t| t.elapsed().as_secs());
                draw_hud(&mut fb, win_w, win_h, armed, connected, shown_fps, throttle, yaw, roll, pitch, frame, trim_roll, trim_pitch, headless, bindings.speed, note, rec_secs);
                // Camera flash: white out the frame briefly right after a capture.
                if snap_note.as_ref().is_some_and(|(_, t)| t.elapsed() < Duration::from_millis(80)) {
                    for px in fb.iter_mut() {
                        *px = 0x00FF_FFFF;
                    }
                }
            }
            if let Some(nw) = &window {
                blit(nw, &fb, win_w, win_h);
            }
        }
    }
    if let Some(rec) = recorder.take() {
        let _ = rec.finish(); // finalize any in-progress recording on exit
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

    // NetworkCapabilities flags.
    const NET_CAP_INTERNET: i32 = 12;
    const NET_CAP_VALIDATED: i32 = 16;

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
        // The drone AP is a *local-only* WiFi: it has no validated internet, while
        // home WiFi does. Prefer the un-validated WiFi so we never bind the drone's
        // UDP to home WiFi after the AP drops and the phone falls back. Fall back to
        // any WiFi if none stands out (e.g. cold start with only the drone AP up).
        let mut chosen: Option<JObject> = None;
        let mut chosen_local = false;
        for i in 0..len {
            let net = env.get_object_array_element(&networks, i)?;
            let caps = env
                .call_method(&cm, "getNetworkCapabilities", "(Landroid/net/Network;)Landroid/net/NetworkCapabilities;", &[(&net).into()])?
                .l()?;
            if caps.is_null() {
                continue;
            }
            let is_wifi = env.call_method(&caps, "hasTransport", "(I)Z", &[JValue::Int(TRANSPORT_WIFI)])?.z()?;
            if !is_wifi {
                continue;
            }
            let validated = env.call_method(&caps, "hasCapability", "(I)Z", &[JValue::Int(NET_CAP_VALIDATED)])?.z()?;
            let internet = env.call_method(&caps, "hasCapability", "(I)Z", &[JValue::Int(NET_CAP_INTERNET)])?.z()?;
            log::info!("bind_to_wifi: wifi net[{i}] internet={internet} validated={validated}");
            if !validated && !chosen_local {
                chosen = Some(net); // the drone's local-only AP — prefer it
                chosen_local = true;
            } else if chosen.is_none() {
                chosen = Some(net); // fallback
            }
        }
        match chosen {
            Some(net) => {
                let ok = env
                    .call_method(&cm, "bindProcessToNetwork", "(Landroid/net/Network;)Z", &[(&net).into()])?
                    .z()?;
                log::info!("bind_to_wifi -> {ok} (local-only AP={chosen_local})");
                Ok(ok)
            }
            None => {
                log::info!("bind_to_wifi: no wifi network available");
                Ok(false)
            }
        }
    })();
    let _ = env.exception_clear();
    res.unwrap_or_else(|e| {
        log::error!("bind_to_wifi error: {e:?}");
        false
    })
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

/// App-private files dir (for persisting bindings), via Context.getFilesDir().
fn files_dir() -> Option<String> {
    let ctx = ndk_context::android_context();
    let vm = unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) }.ok()?;
    let mut env = vm.attach_current_thread().ok()?;
    let activity = unsafe { JObject::from_raw(ctx.context().cast()) };
    let r = (|| -> jni::errors::Result<String> {
        let file = env.call_method(&activity, "getFilesDir", "()Ljava/io/File;", &[])?.l()?;
        let path = env.call_method(&file, "getAbsolutePath", "()Ljava/lang/String;", &[])?.l()?;
        let js = jni::objects::JString::from(path);
        let s = env.get_string(&js)?;
        Ok(s.into())
    })();
    let _ = env.exception_clear();
    r.ok()
}

/// App-private *external* files dir (for snapshots), via getExternalFilesDir(null).
/// No storage permission needed; reachable via a file manager or `adb pull`.
fn external_files_dir() -> Option<String> {
    let ctx = ndk_context::android_context();
    let vm = unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) }.ok()?;
    let mut env = vm.attach_current_thread().ok()?;
    let activity = unsafe { JObject::from_raw(ctx.context().cast()) };
    let r = (|| -> jni::errors::Result<String> {
        let null = JObject::null();
        let file = env
            .call_method(&activity, "getExternalFilesDir", "(Ljava/lang/String;)Ljava/io/File;", &[(&null).into()])?
            .l()?;
        if file.is_null() {
            return Err(jni::errors::Error::NullPtr("getExternalFilesDir"));
        }
        let path = env.call_method(&file, "getAbsolutePath", "()Ljava/lang/String;", &[])?.l()?;
        let js = jni::objects::JString::from(path);
        let s = env.get_string(&js)?;
        Ok(s.into())
    })();
    let _ = env.exception_clear();
    r.ok()
}

/// What a tap in the settings overlay hit.
enum SettingsHit {
    Row(settings::Action),
    ThrottleToggle,
    Reset,
    Done,
}

fn settings_row_y(i: usize, h: usize) -> usize {
    h / 6 + i * (h / 16)
}
fn done_btn(w: usize, h: usize) -> (usize, usize, usize, usize) {
    (w - w / 12 - w / 4, h - h / 8, w / 4, h / 18)
}
fn reset_btn(w: usize, h: usize) -> (usize, usize, usize, usize) {
    (w / 12, h - h / 8, w / 4, h / 18)
}

fn settings_hit(px: usize, py: usize, w: usize, h: usize) -> Option<SettingsHit> {
    let inside = |(x, y, bw, bh): (usize, usize, usize, usize)| px >= x && px < x + bw && py >= y && py < y + bh;
    if inside(done_btn(w, h)) {
        return Some(SettingsHit::Done);
    }
    if inside(reset_btn(w, h)) {
        return Some(SettingsHit::Reset);
    }
    let rh = h / 16;
    for (i, a) in settings::ACTIONS.iter().enumerate() {
        let ry = settings_row_y(i, h);
        if px > w / 12 && px < w - w / 12 && py >= ry && py < ry + rh {
            return Some(SettingsHit::Row(*a));
        }
    }
    // Throttle-source row (after the action rows).
    let ry = settings_row_y(settings::ACTIONS.len(), h);
    if px > w / 12 && px < w - w / 12 && py >= ry && py < ry + rh {
        return Some(SettingsHit::ThrottleToggle);
    }
    None
}

/// Full-screen key-mapping overlay.
fn draw_settings(fb: &mut [u32], w: usize, h: usize, b: &settings::Bindings, listening: Option<settings::Action>) {
    let mut c = hud::Canvas { buf: fb, w, h };
    c.panel(0, 0, w, h, 235); // dark translucent over the video
    let s = (w / 360).max(2);
    let g = 8 * s;
    c.glow_text(w / 12, h / 12, "KEY MAPPING", hud::CYAN, s + 1);
    let rh = h / 16;
    for (i, a) in settings::ACTIONS.iter().enumerate() {
        let ry = settings_row_y(i, h) + (rh - g) / 2;
        let lit = listening == Some(*a);
        let col = if lit { hud::MAGENTA } else { hud::CYAN };
        c.glow_text(w / 12, ry, a.label(), col, s);
        let val = if lit { "PRESS BUTTON".to_string() } else { settings::button_name(b.get(*a)) };
        c.glow_text(w - w / 12 - val.len() * g, ry, &val, col, s);
    }
    // Throttle-source toggle row.
    let ry = settings_row_y(settings::ACTIONS.len(), h) + (rh - g) / 2;
    c.glow_text(w / 12, ry, "THROTTLE", hud::CYAN, s);
    let tval = if b.throttle_triggers { "TRIGGERS" } else { "L-STICK Y" };
    c.glow_text(w - w / 12 - tval.len() * g, ry, tval, hud::AMBER, s);
    // DONE / RESET buttons
    for (rect, label, col) in [(done_btn(w, h), "DONE", hud::GREEN), (reset_btn(w, h), "RESET", hud::AMBER)] {
        let (bx, by, bw, bh) = rect;
        c.hline(bx, by, bw, col);
        c.hline(bx, by + bh, bw, col);
        c.vline(bx, by, bh, col);
        c.vline(bx + bw, by, bh, col);
        c.glow_text(bx + bw.saturating_sub(label.len() * g) / 2, by + bh.saturating_sub(g) / 2, label, col, s);
    }
    c.glow_text(w / 12, h / 12 + (s + 1) * 8 + s * 2, "tap a row, then press a controller button", hud::CYAN, s.max(2));
}

fn handle_input(event: &InputEvent, pad: &mut Pad, b: &settings::Bindings) -> InputStatus {
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
                // Analog triggers (some pads report these on Gas/Brake instead).
                pad.rtrig = p.axis_value(Axis::Rtrigger).max(p.axis_value(Axis::Gas));
                pad.ltrig = p.axis_value(Axis::Ltrigger).max(p.axis_value(Axis::Brake));
                return InputStatus::Handled;
            }
            // Touch: record tap-down screen coords for HUD button hit-testing.
            if u32::from(m.source()) & SOURCE_CLASS_POINTER != 0
                && matches!(m.action(), MotionAction::Down | MotionAction::PointerDown)
                && m.pointer_count() > 0
            {
                let p = m.pointer_at_index(0);
                pad.tap = Some((p.raw_x(), p.raw_y()));
                return InputStatus::Handled;
            }
            InputStatus::Unhandled
        }
        InputEvent::KeyEvent(k) => {
            use settings::Action::*;
            let down = matches!(k.action(), KeyAction::Down);
            let edge = down && k.repeat_count() == 0;
            let kc = u32::from(k.key_code());
            if down {
                pad.captured_key = Some(kc);
            }
            if kc == b.get(Arm) && edge { pad.arm_toggle = true; }
            if kc == b.get(Headless) && edge { pad.headless_toggle = true; }
            if kc == b.get(TrimReset) && edge { pad.trim_reset = true; }
            if kc == b.get(Takeoff) { pad.takeoff = down; }
            if kc == b.get(Land) { pad.land = down; }
            if kc == b.get(Flip) { pad.flip = down; }
            if kc == b.get(Calibrate) { pad.calibrate = down; }
            if kc == b.get(Emergency) { pad.emergency = down; }
            if kc == b.get(Emergency) && edge { pad.estop = true; }
            if kc == b.get(Speed) && edge { pad.speed_cycle = true; }
            if kc == 104 { pad.l2 = down; } // ButtonL2 (digital trigger / yaw-trim left)
            if kc == 105 { pad.r2 = down; } // ButtonR2 (digital trigger / yaw-trim right)
            InputStatus::Handled
        }
        _ => InputStatus::Unhandled,
    }
}

/// "KEY MAP" button rect — bottom-center, between the stick boxes (KEY MAP above SIM).
fn menu_btn(w: usize, h: usize) -> (usize, usize, usize, usize) {
    let s = (w / 360).max(2);
    let bs = w / 4; // stick-box size (matches draw_hud)
    let band_center = (h - h / 18) - 8 * s - bs / 2; // stick-box vertical center
    let bw = w / 4;
    let bh = h / 26;
    (w / 2 - bw / 2, band_center - bh - h / 120, bw, bh)
}

/// "SIM" button rect — opens the virtual-drone preview (disarmed). Below KEY MAP.
fn sim_btn(w: usize, h: usize) -> (usize, usize, usize, usize) {
    let (mx, my, bw, bh) = menu_btn(w, h);
    (mx, my + bh + h / 80, bw, bh)
}

/// "EXIT" button rect for the preview screen (bottom-center).
fn exit_btn(w: usize, h: usize) -> (usize, usize, usize, usize) {
    (w / 2 - w / 8, h - h / 8, w / 4, h / 18)
}

/// "PHOTO" snapshot button rect — upper-right, clear of the status panel and sticks.
fn photo_btn(w: usize, h: usize) -> (usize, usize, usize, usize) {
    let bw = w / 5;
    let bh = h / 22;
    (w - w / 18 - bw, h / 4, bw, bh)
}

/// "REC" video button rect — just below the PHOTO button.
fn rec_btn(w: usize, h: usize) -> (usize, usize, usize, usize) {
    let (bx, by, bw, bh) = photo_btn(w, h);
    (bx, by + bh + h / 60, bw, bh)
}

/// Write the latest raw JPEG frame to `<dir>/snap_<epoch-ms>.jpg`. Returns the file name.
fn save_snapshot(dir: &str, jpeg: &[u8]) -> std::io::Result<String> {
    std::fs::create_dir_all(dir)?;
    let ms = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis()).unwrap_or(0);
    let name = format!("snap_{ms}.jpg");
    std::fs::write(format!("{dir}/{name}"), jpeg)?;
    Ok(name)
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
    speed: u8,
    snap_note: Option<&str>,
    rec_secs: Option<u64>,
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
    c.glow_text(tx + 14 * g, row2, &format!("SPD {}", settings::speed_name(speed)), hud::CYAN, s);
    if headless {
        c.glow_text(x1 - 9 * g, row2, "HEADLESS", hud::MAGENTA, s);
    }

    // KEY MAP + SIM buttons (disarmed only).
    if !armed {
        for (rect, label) in [(menu_btn(w, h), "KEY MAP"), (sim_btn(w, h), "SIM")] {
            let (bx, by, bw, bh) = rect;
            c.hline(bx, by, bw, hud::CYAN);
            c.hline(bx, by + bh, bw, hud::CYAN);
            c.vline(bx, by, bh, hud::CYAN);
            c.vline(bx + bw, by, bh, hud::CYAN);
            c.glow_text(bx + bw.saturating_sub(label.len() * g) / 2, by + bh.saturating_sub(g) / 2, label, hud::CYAN, s);
        }
    }

    // stick boxes near the bottom safe corners
    let bs = w / 4;
    c.stick_box(x0 + 2 * s, y1 - bs - g, bs, yaw, throttle, hud::MAGENTA);
    c.glow_text(x0 + 2 * s, y1 - g + s, "YAW/THR", hud::MAGENTA, s.max(2));
    c.stick_box(x1 - bs - 2 * s, y1 - bs - g, bs, roll, pitch, hud::CYAN);
    c.glow_text(x1 - 8 * g, y1 - g + s, "ROL/PIT", hud::CYAN, s.max(2));

    // Tappable PHOTO + REC buttons (only while we have video).
    if connected {
        let btn = |c: &mut hud::Canvas, rect: (usize, usize, usize, usize), label: &str, col: u32| {
            let (bx, by, bw, bh) = rect;
            c.panel(bx, by, bw, bh, 200);
            c.hline(bx, by, bw, col);
            c.hline(bx, by + bh, bw, col);
            c.vline(bx, by, bh, col);
            c.vline(bx + bw, by, bh, col);
            c.glow_text(bx + bw.saturating_sub(label.len() * g) / 2, by + bh.saturating_sub(g) / 2, label, col, s);
        };
        btn(&mut c, photo_btn(w, h), "PHOTO", hud::GREEN);
        // REC button turns red + shows elapsed time while recording.
        let recording = rec_secs.is_some();
        let rec_col = if recording { hud::RED } else { hud::CYAN };
        let rec_label = if let Some(secs) = rec_secs {
            format!("REC {:02}:{:02}", secs / 60, secs % 60)
        } else {
            "REC".to_string()
        };
        btn(&mut c, rec_btn(w, h), &rec_label, rec_col);
        // Blinking record dot in the status panel while recording.
        if recording && (frame / 12).is_multiple_of(2) {
            c.fill(tx, row0 + g + 2 * s, 2 * s, 2 * s, hud::RED);
        }
        if let Some(note) = snap_note {
            let (pbx, _, pbw, _) = photo_btn(w, h);
            let (_, rby, _, rbh) = rec_btn(w, h);
            let nw = note.len() * g;
            c.glow_text(pbx + pbw.saturating_sub(nw) / 2, rby + rbh + s * 2, note, hud::GREEN, s);
        }
    }

    // Loud, blinking "LINK LOST" banner when no video is arriving — fly back!
    // Tapping anywhere on the feed reconnects, so prompt for it right here.
    if !connected && (frame / 18).is_multiple_of(2) {
        let bscale = s + 1;
        let msg = "LINK LOST";
        let tw = msg.len() * 8 * bscale;
        let bx = w.saturating_sub(tw) / 2;
        let by = h * 2 / 5;
        let hint = "TAP TO RECONNECT";
        let hw = hint.len() * 8 * s;
        let bw = tw.max(hw);
        c.panel((w - bw) / 2 - 6, by.saturating_sub(6), bw + 12, 8 * bscale + g + 12, 210);
        c.glow_text(bx, by, msg, hud::RED, bscale);
        c.glow_text(w.saturating_sub(hw) / 2, by + 8 * bscale + s * 2, hint, hud::CYAN, s);
    }
}

/// Virtual-drone preview: a neon wireframe quad seen from behind/above, "flown"
/// with the mapped controls (disarmed; nothing sent). Roll/pitch tilt it, yaw
/// spins it, throttle/takeoff/land set altitude, flip rolls it a full 360°.
#[allow(clippy::too_many_arguments)]
fn draw_preview(
    fb: &mut [u32],
    w: usize,
    h: usize,
    roll: u8,
    pitch: u8,
    yaw: u8,
    throttle: u8,
    pad: &Pad,
    heading: f32,
    alt: f32,
    flip_roll: f32,
    flip_pitch: f32,
    spin: f32,
    killed: bool,
) {
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, TAU};
    let mut c = hud::Canvas { buf: fb, w, h };
    c.panel(0, 0, w, h, 215);
    let s = (w / 360).max(2);
    let g = 8 * s;
    c.glow_text(w / 12, h / 12, "VIRTUAL DRONE - SIM", hud::MAGENTA, s);
    // Readouts up top, clear of the EXIT button.
    c.glow_text(w / 12, h / 12 + g + s * 2, &format!("THR{throttle:3} YAW{yaw:3} ROL{roll:3} PIT{pitch:3}"), hud::CYAN, s);
    c.glow_text(w - w / 4, h / 12 + g + s * 2, &format!("ALT{:3.0}%", (alt * 100.0).min(120.0)), hud::CYAN, s);
    let mut bx = w / 12;
    for (lbl, on, col) in [
        ("TAKEOFF", pad.takeoff, hud::GREEN),
        ("LAND", pad.land, hud::AMBER),
        ("FLIP", pad.flip || flip_roll.abs() + flip_pitch.abs() > 0.05, hud::MAGENTA),
        ("CALIB", pad.calibrate, hud::CYAN),
        ("KILLSWITCH", killed, hud::RED),
    ] {
        c.glow_text(bx, h / 12 + 2 * g + s * 4, lbl, if on { col } else { 0x0033_3333 }, s);
        bx += (lbl.len() + 1) * g;
    }

    // Rotors spin while flying (alt/takeoff/flip), static when shut off or killed.
    // With the motors off the drone can't manoeuvre: ignore roll/pitch (stays level).
    let motors_on = !killed && (alt > 0.02 || pad.takeoff || flip_roll.abs() + flip_pitch.abs() > 0.05);
    let (roll, pitch) = if motors_on { (roll, pitch) } else { (128u8, 128u8) };

    // Behind/above pseudo-3D: roll about forward (Y), pitch about side (X), yaw about up (Z).
    let ground = h as f32 * 0.62;
    let cxf = w as f32 / 2.0;
    let drone_y = ground - alt * h as f32 * 0.30;
    let arm = w as f32 / 4.0;
    let rotor = ((5.0 + (throttle as f32 / 255.0) * 12.0) * s as f32) as i32;
    let (sr, cr) = ((roll as f32 - 128.0) / 128.0 * 0.6 + flip_roll).sin_cos();
    // Pitch tilt inverted in the sim view only (flight control unchanged).
    let (sp, cp) = (-(pitch as f32 - 128.0) / 128.0 * 0.6 + flip_pitch).sin_cos();
    let (yh_s, yh_c) = heading.sin_cos();
    let (scam, ccam) = 0.5f32.sin_cos(); // camera look-down
    let project = |bx: f32, by: f32| -> (i32, i32) {
        let (mut x, mut y, mut z) = (bx, by, 0.0f32);
        let (nx, nz) = (x * cr + z * sr, -x * sr + z * cr); // roll about Y
        x = nx;
        z = nz;
        let (ny, nz2) = (y * cp - z * sp, y * sp + z * cp); // pitch about X
        y = ny;
        z = nz2;
        let (nx2, ny2) = (x * yh_c - y * yh_s, x * yh_s + y * yh_c); // yaw about Z
        x = nx2;
        y = ny2;
        ((cxf + x) as i32, (drone_y - z * ccam - y * scam) as i32)
    };

    // ground reference line
    c.hline(w / 8, ground as usize, w * 3 / 4, 0x0020_3020);

    // Fuselage corners (computed first so the arms can attach to them): a pointed
    // body whose nose marks the front — no separate orientation marker needed.
    // Hexagonal body, elongated with a pointed nose and tail. The four side corners
    // sit on the diagonals (equal |x|,|y|) so each arm is collinear with the
    // centre->rotor line: the arms read as a single X, but the crossing is hidden
    // by the body — no lines are drawn inside the fuselage.
    let cw = arm * 0.24; // corner half-width (= corner depth, keeps it on the diagonal)
    let bp = [
        project(0.0, arm * 0.80), // 0 nose
        project(cw, cw),          // 1 front-right
        project(cw, -cw),         // 2 rear-right
        project(0.0, -arm * 0.36), // 3 tail
        project(-cw, -cw),        // 4 rear-left
        project(-cw, cw),         // 5 front-left
    ];

    // Arms + rotors. All one colour at full brightness — the pointed fuselage shows
    // which way the drone faces. Each arm starts at its fuselage corner.
    let (bright, dim) = (hud::CYAN, 0x0020_6070);
    let arm_root = [1usize, 5, 4, 2]; // rotor k -> fuselage corner (FR, FL, RL, RR)
    for k in 0..4 {
        let a = FRAC_PI_4 + k as f32 * FRAC_PI_2;
        let (rx, ry) = project(a.cos() * arm, a.sin() * arm);
        let (sx, sy) = bp[arm_root[k]];
        c.line(sx, sy, rx, ry, bright);
        let r = rotor as f32;
        if motors_on {
            // Faint disc + three radial blades. Radial spokes (120° apart, not full
            // diameters) keep the symmetry period at 120°, which the per-frame spin
            // increment never reaches — so it never strobes to a standstill. The
            // leading blade is brighter to make the spin direction unambiguous.
            for seg in 0..16 {
                let a0 = seg as f32 * (TAU / 16.0);
                let a1 = a0 + TAU / 16.0;
                c.line(rx + (r * a0.cos()) as i32, ry + (r * a0.sin()) as i32,
                       rx + (r * a1.cos()) as i32, ry + (r * a1.sin()) as i32, dim);
            }
            for b in 0..3 {
                let a = spin + b as f32 * (TAU / 3.0);
                let col = if b == 0 { bright } else { dim };
                c.line(rx, ry, rx + (r * a.cos()) as i32, ry + (r * a.sin()) as i32, col);
            }
        } else {
            // Stopped: a single two-blade prop at rest.
            let (dx, dy) = ((r * FRAC_PI_4.cos()) as i32, (r * FRAC_PI_4.sin()) as i32);
            c.line(rx - dx, ry - dy, rx + dx, ry + dy, bright);
            c.line(rx - dx, ry + dy, rx + dx, ry - dy, dim);
        }
    }

    // Fuselage outline.
    for i in 0..bp.len() {
        let (x0, y0) = bp[i];
        let (x1, y1) = bp[(i + 1) % bp.len()];
        c.line(x0, y0, x1, y1, bright);
    }

    // EXIT button
    let (ex, ey, ew, eh) = exit_btn(w, h);
    c.hline(ex, ey, ew, hud::RED);
    c.hline(ex, ey + eh, ew, hud::RED);
    c.vline(ex, ey, eh, hud::RED);
    c.vline(ex + ew, ey, eh, hud::RED);
    c.glow_text(ex + ew.saturating_sub(4 * g) / 2, ey + eh.saturating_sub(g) / 2, "EXIT", hud::RED, s);
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
