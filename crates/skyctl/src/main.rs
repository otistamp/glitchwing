//! Live connectivity probe for the DRCX5.
//!
//! Connects to the drone (default `192.168.4.153`), requests the video stream,
//! and for a few seconds reports how many MJPEG frames arrive, the frame rate,
//! and saves sample frames to `live_frames/`. Stays **disarmed** the whole time,
//! so only the harmless idle keepalive (`0xAA…0x55`) is sent — no motor commands.
//!
//! Usage: cargo run -p skyctl -- [seconds]   (default 8)

use std::fs;
use std::time::{Duration, Instant};

use net::{DroneLink, LinkConfig};

fn main() {
    let secs: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(8);

    let cfg = LinkConfig::default();
    println!("skyctl: connecting to drone");
    println!("  control -> {}", cfg.control_addr);
    println!("  video   -> {} (requesting stream)", cfg.video_addr);
    println!("  staying DISARMED (idle keepalive only — no motor commands)\n");

    let link = match DroneLink::start(cfg) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("failed to start link: {e}");
            std::process::exit(1);
        }
    };

    fs::create_dir_all("live_frames").ok();
    let start = Instant::now();
    let deadline = start + Duration::from_secs(secs);
    let (mut count, mut saved, mut bytes, mut first_at) = (0u32, 0u32, 0usize, None);

    while Instant::now() < deadline {
        match link.frames.recv_timeout(Duration::from_millis(250)) {
            Ok(frame) => {
                count += 1;
                bytes += frame.len();
                if first_at.is_none() {
                    first_at = Some(start.elapsed());
                    println!("first frame after {:?} ({} bytes)", first_at.unwrap(), frame.len());
                }
                if saved < 5 && count % 10 == 1 {
                    let path = format!("live_frames/live_{count:03}.jpg");
                    if fs::write(&path, &frame).is_ok() {
                        println!("  saved {path} ({} bytes)", frame.len());
                        saved += 1;
                    }
                }
            }
            Err(_) => print!("."),
        }
    }
    link.stop();

    let elapsed = start.elapsed().as_secs_f64();
    println!("\n\n=== probe summary ===");
    println!("frames received : {count}");
    if count > 0 {
        println!("frame rate      : {:.1} fps", count as f64 / elapsed);
        println!("avg frame size  : {} bytes", bytes / count as usize);
        println!("\nlive video is WORKING — sample frames in live_frames/");
    } else {
        println!("\nNO video frames received. Checklist:");
        println!("  - Is the Mac joined to the drone's WiFi (drone powered on)?");
        println!("  - Can you ping 192.168.4.153 ?");
        println!("  - Did the stock app need to be closed first (single client)?");
    }
}
