//! Cyberpunk HUD: neon text, glow, scanlines, brackets, bars and stick boxes,
//! rendered onto a `0x00RRGGBB` framebuffer with an embedded 8x8 font.

use font8x8::{UnicodeFonts, BASIC_FONTS};

pub const CYAN: u32 = 0x0000_F0FF;
pub const MAGENTA: u32 = 0x00FF_2BD6;
pub const GREEN: u32 = 0x0039_FF14;
pub const RED: u32 = 0x00FF_1133;
pub const AMBER: u32 = 0x00FF_B000;
pub const DARK: u32 = 0x0005_0A12;

fn ch(v: u32, sh: u32) -> u32 {
    (v >> sh) & 0xFF
}

/// Alpha-blend `fg` over `bg` (`a` = 0..=255).
fn blend(bg: u32, fg: u32, a: u32) -> u32 {
    let inv = 255 - a;
    let r = (ch(fg, 16) * a + ch(bg, 16) * inv) / 255;
    let g = (ch(fg, 8) * a + ch(bg, 8) * inv) / 255;
    let b = (ch(fg, 0) * a + ch(bg, 0) * inv) / 255;
    (r << 16) | (g << 8) | b
}

/// A borrowed RGB framebuffer with cyberpunk drawing primitives.
pub struct Canvas<'a> {
    pub buf: &'a mut [u32],
    pub w: usize,
    pub h: usize,
}

impl Canvas<'_> {
    #[inline]
    fn put(&mut self, x: usize, y: usize, color: u32) {
        if x < self.w && y < self.h {
            self.buf[y * self.w + x] = color;
        }
    }

    #[inline]
    fn blend_px(&mut self, x: usize, y: usize, color: u32, a: u32) {
        if x < self.w && y < self.h {
            let i = y * self.w + x;
            self.buf[i] = blend(self.buf[i], color, a);
        }
    }

    /// CRT scanlines: darken every other row.
    pub fn scanlines(&mut self) {
        for y in (0..self.h).step_by(2) {
            for x in 0..self.w {
                let i = y * self.w + x;
                self.buf[i] = blend(self.buf[i], 0, 60);
            }
        }
    }

    /// Translucent dark panel for HUD readability.
    pub fn panel(&mut self, x: usize, y: usize, w: usize, h: usize, a: u32) {
        for yy in y..(y + h) {
            for xx in x..(x + w) {
                self.blend_px(xx, yy, DARK, a);
            }
        }
    }

    /// Filled rectangle.
    pub fn fill(&mut self, x: usize, y: usize, w: usize, h: usize, color: u32) {
        for yy in y..(y + h) {
            for xx in x..(x + w) {
                self.put(xx, yy, color);
            }
        }
    }

    pub fn hline(&mut self, x: usize, y: usize, len: usize, color: u32) {
        for i in 0..len {
            self.put(x + i, y, color);
        }
    }

    pub fn vline(&mut self, x: usize, y: usize, len: usize, color: u32) {
        for i in 0..len {
            self.put(x, y + i, color);
        }
    }

    /// Neon double border with cyan corner brackets; `color` tints the inner line.
    pub fn neon_frame(&mut self, color: u32) {
        let (w, h) = (self.w, self.h);
        for x in 0..w {
            self.put(x, 0, color);
            self.put(x, h - 1, color);
            self.blend_px(x, 1, color, 120);
            self.blend_px(x, h - 2, color, 120);
        }
        for y in 0..h {
            self.put(0, y, color);
            self.put(w - 1, y, color);
            self.blend_px(1, y, color, 120);
            self.blend_px(w - 2, y, color, 120);
        }
        // corner brackets
        let n = 14;
        for (cx, cy, dx, dy) in [(2, 2, 1i32, 1i32), (w - 3, 2, -1, 1), (2, h - 3, 1, -1), (w - 3, h - 3, -1, -1)] {
            for i in 0..n {
                self.put((cx as i32 + dx * i) as usize, cy, CYAN);
                self.put(cx, (cy as i32 + dy * i) as usize, CYAN);
            }
        }
    }

    /// High-contrast neon text: an outer colored glow, a solid black outline for
    /// legibility on any background, then a bright core.
    pub fn glow_text(&mut self, x: usize, y: usize, text: &str, color: u32, scale: usize) {
        let at = |c: &mut Self, ox: i32, oy: i32, col: u32, a: u32| {
            c.text_raw((x as i32 + ox * scale as i32).max(0) as usize,
                       (y as i32 + oy * scale as i32).max(0) as usize, text, col, scale, a);
        };
        // outer colored bloom (radius 2)
        for (ox, oy) in [(0, -2), (0, 2), (-2, 0), (2, 0)] {
            at(self, ox, oy, color, 45);
        }
        // solid black outline (8-neighbour, radius 1) — this is what makes it readable
        for (ox, oy) in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)] {
            at(self, ox, oy, 0x0000_0000, 255);
        }
        // bright core
        let core = blend(color, 0x00FF_FFFF, 70); // slightly brighten the neon
        self.text_raw(x, y, text, core, scale, 255);
    }

    fn text_raw(&mut self, x: usize, y: usize, text: &str, color: u32, scale: usize, a: u32) {
        let mut cx = x;
        for c in text.chars() {
            if let Some(glyph) = BASIC_FONTS.get(c) {
                for (row, bits) in glyph.iter().enumerate() {
                    for col in 0..8 {
                        if bits & (1 << col) != 0 {
                            for sy in 0..scale {
                                for sx in 0..scale {
                                    self.blend_px(cx + col * scale + sx, y + row * scale + sy, color, a);
                                }
                            }
                        }
                    }
                }
            }
            cx += 8 * scale; // leave room for the glyph outline
        }
    }

    /// Horizontal neon bar: dark track with a filled portion (`frac` 0..=1) and a
    /// center tick (useful for centered axes like throttle).
    pub fn bar(&mut self, x: usize, y: usize, w: usize, h: usize, frac: f32, color: u32) {
        for yy in y..(y + h) {
            for xx in x..(x + w) {
                self.blend_px(xx, yy, DARK, 200);
            }
        }
        let fill = ((frac.clamp(0.0, 1.0)) * w as f32) as usize;
        for yy in y..(y + h) {
            for xx in x..(x + fill) {
                self.put(xx, yy, color);
            }
        }
        // center tick
        for yy in y..(y + h) {
            self.blend_px(x + w / 2, yy, CYAN, 180);
        }
    }

    /// Stick-position box: outlined square with a neon dot at (`ax`,`ay`) in 0..=255,
    /// y drawn inverted (255 = top), like an FPV transmitter OSD.
    pub fn stick_box(&mut self, x: usize, y: usize, size: usize, ax: u8, ay: u8, color: u32) {
        self.hline(x, y, size, blend(color, 0, 80));
        self.hline(x, y + size, size, blend(color, 0, 80));
        self.vline(x, y, size, blend(color, 0, 80));
        self.vline(x + size, y, size, blend(color, 0, 80));
        // faint center crosshair
        self.hline(x, y + size / 2, size, blend(color, 0, 160));
        self.vline(x + size / 2, y, size, blend(color, 0, 160));
        let cx = (x + (ax as usize * size) / 255) as i32;
        let cy = (y + ((255 - ay as usize) * size) / 255) as i32;
        for oy in -1i32..=1 {
            for ox in -1i32..=1 {
                if cx + ox >= 0 && cy + oy >= 0 {
                    self.put((cx + ox) as usize, (cy + oy) as usize, color);
                }
            }
        }
    }
}
