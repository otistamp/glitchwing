//! Generates the Glitchwing launcher icon as an Android adaptive icon: a neon
//! wireframe quad (foreground) over a dark scanline/grid field (background), so
//! it fills the launcher's circle instead of sitting as a square inside it.
//!
//!   cargo run -p hud --example icon -- crates/android/res
//!
//! Writes mipmap/ic_launcher_foreground.png, mipmap/ic_launcher_background.png,
//! and mipmap/ic_launcher.png (a combined legacy fallback). The adaptive XML
//! (mipmap-anydpi-v26/ic_launcher.xml) is committed alongside, not generated.

use std::fs::{create_dir_all, File};
use std::io::BufWriter;
use std::path::Path;

use hud::{Canvas, CYAN, MAGENTA};

const BG: u32 = 0x0005_070C; // near-black with a faint blue cast
const GRID: u32 = 0x000C_1A1E; // dim grid line
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
        c.line(cx, cy, rx, ry, color);
        ring(c, rx, ry, rr, color);
    }
    let b = (arm * 0.30) as i32;
    c.line(cx, cy - b, cx + b, cy, color);
    c.line(cx + b, cy, cx, cy + b, color);
    c.line(cx, cy + b, cx - b, cy, color);
    c.line(cx - b, cy, cx, cy - b, color);
    c.line(cx, cy, cx, cy - (arm * 0.95) as i32, nose);
}

/// Draw the glitchy RGB-split drone (used by foreground + combined).
fn draw_drone(c: &mut Canvas, s: i32) {
    let (cx, cy) = (s / 2, s / 2);
    let arm = s as f32 * 0.23;
    let rr = s as f32 * 0.09;
    let g = (s as f32 * 0.02) as i32;
    quad(c, cx - g, cy, arm, rr, MAGENTA, MAGENTA);
    quad(c, cx + g, cy, arm, rr, CYAN, CYAN);
    quad(c, cx, cy, arm, rr, CORE, MAGENTA);
    quad(c, cx + 1, cy, arm, rr, CORE, MAGENTA);
    // glitch slivers across the body band
    let band = (cy - (arm * 0.4) as i32).max(0) as usize;
    c.hline(s as usize / 6, band, s as usize * 2 / 3, CYAN);
    c.hline(s as usize / 4, band + (s as usize / 36), s as usize / 2, MAGENTA);
    c.hline(s as usize / 6, (cy + (arm * 0.7) as i32) as usize, s as usize / 3, CYAN);
}

/// Dark field with a faint grid + scanlines (the adaptive background, full bleed).
fn draw_field(c: &mut Canvas, s: i32) {
    let step = (s / 9).max(1) as usize;
    let mut x = step;
    while x < s as usize {
        c.vline(x, 0, s as usize, GRID);
        x += step;
    }
    let mut y = step;
    while y < s as usize {
        c.hline(0, y, s as usize, GRID);
        y += step;
    }
    c.scanlines();
}

fn write_png(path: &str, buf: &[u32], s: i32, alpha: impl Fn(u32) -> u8) {
    if let Some(parent) = Path::new(path).parent() {
        create_dir_all(parent).unwrap();
    }
    let file = File::create(path).unwrap();
    let mut enc = png::Encoder::new(BufWriter::new(file), s as u32, s as u32);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    let mut w = enc.write_header().unwrap();
    let mut data = Vec::with_capacity(buf.len() * 4);
    for &px in buf {
        data.push((px >> 16) as u8);
        data.push((px >> 8) as u8);
        data.push(px as u8);
        data.push(alpha(px));
    }
    w.write_image_data(&data).unwrap();
    println!("wrote {path}");
}

fn main() {
    let dir = std::env::args().nth(1).unwrap_or_else(|| "res".into());
    let s: i32 = 432;
    let n = (s * s) as usize;

    // Foreground: drone on pure black -> alpha from non-black so the bg shows through.
    let mut fg = vec![0x0000_0000u32; n];
    draw_drone(&mut Canvas { buf: &mut fg, w: s as usize, h: s as usize }, s);
    write_png(&format!("{dir}/mipmap/ic_launcher_foreground.png"), &fg, s, |px| if px & 0x00FF_FFFF != 0 { 0xFF } else { 0 });

    // Background: full-bleed dark field, fully opaque.
    let mut bg = vec![BG; n];
    draw_field(&mut Canvas { buf: &mut bg, w: s as usize, h: s as usize }, s);
    write_png(&format!("{dir}/mipmap/ic_launcher_background.png"), &bg, s, |_| 0xFF);

    // Combined legacy fallback (pre-API-26): field + drone, opaque.
    let mut combo = vec![BG; n];
    {
        let mut c = Canvas { buf: &mut combo, w: s as usize, h: s as usize };
        draw_field(&mut c, s);
        draw_drone(&mut c, s);
    }
    write_png(&format!("{dir}/mipmap/ic_launcher.png"), &combo, s, |_| 0xFF);
}
