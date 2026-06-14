//! Minimal 8x8 bitmap-font text rendering onto a `0x00RRGGBB` framebuffer.

use font8x8::{UnicodeFonts, BASIC_FONTS};

/// A borrowed RGB framebuffer to draw HUD text onto.
pub struct Canvas<'a> {
    pub buf: &'a mut [u32],
    pub w: usize,
    pub h: usize,
}

impl Canvas<'_> {
    /// Draw `text` at (`x`,`y`) in `color`, with a 1px black shadow for legibility.
    pub fn text(&mut self, x: usize, y: usize, text: &str, color: u32, scale: usize) {
        self.raw(x + 1, y + 1, text, 0x0000_0000, scale);
        self.raw(x, y, text, color, scale);
    }

    fn raw(&mut self, x: usize, y: usize, text: &str, color: u32, scale: usize) {
        let mut cx = x;
        for ch in text.chars() {
            if let Some(glyph) = BASIC_FONTS.get(ch) {
                for (row, bits) in glyph.iter().enumerate() {
                    for col in 0..8 {
                        if bits & (1 << col) != 0 {
                            self.fill_cell(cx + col * scale, y + row * scale, scale, color);
                        }
                    }
                }
            }
            cx += 8 * scale;
        }
    }

    fn fill_cell(&mut self, x: usize, y: usize, scale: usize, color: u32) {
        for sy in 0..scale {
            for sx in 0..scale {
                let (px, py) = (x + sx, y + sy);
                if px < self.w && py < self.h {
                    self.buf[py * self.w + px] = color;
                }
            }
        }
    }
}
