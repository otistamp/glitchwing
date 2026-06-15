//! Generates the Glitchwing launcher icon: a neon wireframe quad with a glitchy
//! RGB channel-split (magenta/cyan offset) on a dark scanline field.
//!
//!   cargo run -p hud --example icon -- crates/android/res/mipmap/ic_launcher.png

use std::fs::{create_dir_all, File};
use std::io::BufWriter;
use std::path::Path;

use hud::{Canvas, CYAN, MAGENTA};

const BG: u32 = 0x0005_070C; // near-black with a faint blue cast
const CORE: u32 = 0x00E6_FFFF; // bright cyan-white

fn ring(c: &mut Canvas, cx: i32, cy: i32, r: f32, color: u32) {
    let n = 28;
    let mut prev = (cx + r as i32, cy);
    for i in 1..=n {
        let a = (i as f32 / n as f32) * std::f32::consts::TAU;
        let p = (cx + (r * a.cos()) as i32, cy + (r * a.sin()) as i32);
        c.line(prev.0, prev.1, p.0, p.1, color);
        prev = p;
    }
}

/// One copy of the quad (arms + rotor rings + body + nose), centred at (cx,cy).
fn quad(c: &mut Canvas, cx: i32, cy: i32, arm: f32, rr: f32, color: u32, nose: u32) {
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};
    for k in 0..4 {
        let a = FRAC_PI_4 + k as f32 * FRAC_PI_2;
        let rx = cx + (a.cos() * arm) as i32;
        let ry = cy + (a.sin() * arm) as i32;
        c.line(cx, cy, rx, ry, color); // arm
        ring(c, rx, ry, rr, color); // rotor disc
    }
    let b = (arm * 0.30) as i32; // body diamond
    c.line(cx, cy - b, cx + b, cy, color);
    c.line(cx + b, cy, cx, cy + b, color);
    c.line(cx, cy + b, cx - b, cy, color);
    c.line(cx - b, cy, cx, cy - b, color);
    c.line(cx, cy, cx, cy - (arm * 0.95) as i32, nose); // forward nose
}

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "ic_launcher.png".into());
    let s: i32 = 432; // launcher icon, scaled down by Android per density
    let mut buf = vec![BG; (s * s) as usize];
    let mut c = Canvas { buf: &mut buf, w: s as usize, h: s as usize };

    let cx = s / 2;
    let cy = s / 2;
    let arm = s as f32 * 0.21; // kept inside the adaptive-icon safe zone
    let rr = s as f32 * 0.085;
    let g = (s as f32 * 0.02) as i32; // channel-split glitch offset

    // RGB-split: a magenta copy left, a cyan copy right, a bright core centred.
    quad(&mut c, cx - g, cy, arm, rr, MAGENTA, MAGENTA);
    quad(&mut c, cx + g, cy, arm, rr, CYAN, CYAN);
    quad(&mut c, cx, cy, arm, rr, CORE, MAGENTA);
    quad(&mut c, cx + 1, cy, arm, rr, CORE, MAGENTA); // double the core to bolden

    // Glitch slivers: a few offset bright scanlines through the body band.
    let band = (cy - (arm * 0.4) as i32).max(0) as usize;
    c.hline(s as usize / 6, band, s as usize * 2 / 3, CYAN);
    c.hline(s as usize / 4, band + (s as usize / 36), s as usize / 2, MAGENTA);
    c.hline(0, (cy + (arm * 0.7) as i32) as usize, s as usize / 3, CYAN);

    c.scanlines(); // CRT lines over everything

    // Encode RGBA PNG.
    if let Some(parent) = Path::new(&path).parent() {
        create_dir_all(parent).unwrap();
    }
    let file = File::create(&path).unwrap();
    let mut enc = png::Encoder::new(BufWriter::new(file), s as u32, s as u32);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    let mut writer = enc.write_header().unwrap();
    let mut data = Vec::with_capacity((s * s * 4) as usize);
    for &px in buf.iter() {
        data.push((px >> 16) as u8);
        data.push((px >> 8) as u8);
        data.push(px as u8);
        data.push(0xFF);
    }
    writer.write_image_data(&data).unwrap();
    println!("wrote {path} ({s}x{s})");
}
