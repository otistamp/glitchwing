//! Minimal Motion-JPEG AVI muxer.
//!
//! The DRCX5 video stream is already a sequence of JPEG frames, so "recording"
//! is just muxing those frames verbatim into an AVI container with the `MJPG`
//! codec — no re-encode. The result plays in VLC / ffmpeg / most players.
//!
//! Writes the header with placeholder sizes up front, appends each frame as a
//! `00dc` chunk, then on [`AviWriter::finish`] writes the `idx1` index and seeks
//! back to patch the RIFF size, frame count, stream length and `movi` size.

use std::io::{self, Seek, SeekFrom, Write};

/// Streaming MJPEG-in-AVI writer over any seekable sink.
pub struct AviWriter<W: Write + Seek> {
    w: W,
    /// (offset from the `movi` FOURCC, payload size) per frame — for `idx1`.
    frames: Vec<(u32, u32)>,
    riff_size_pos: u64,
    total_frames_pos: u64,
    stream_len_pos: u64,
    movi_size_pos: u64,
    movi_fourcc_pos: u64,
    movi_bytes: u32, // running size of the movi LIST payload (starts at 4 for "movi")
}

impl<W: Write + Seek> AviWriter<W> {
    /// Begin an AVI for `width`x`height` MJPEG frames at `fps`.
    pub fn new(mut w: W, width: u32, height: u32, fps: u32) -> io::Result<Self> {
        let fps = fps.max(1);
        // hdrl is fixed-size: hdrl(4) + avih(8+56) + strl LIST(8 + 4 + strh(8+56) + strf(8+40))
        let hdrl_size: u32 = 4 + 64 + (8 + 4 + 64 + 48);
        let strl_size: u32 = 4 + 64 + 48;

        w.write_all(b"RIFF")?;
        let riff_size_pos = w.stream_position()?;
        w.write_all(&0u32.to_le_bytes())?; // RIFF size (patched in finish)
        w.write_all(b"AVI ")?;

        // --- hdrl ---
        w.write_all(b"LIST")?;
        w.write_all(&hdrl_size.to_le_bytes())?;
        w.write_all(b"hdrl")?;

        // avih (MainAVIHeader, 56 bytes)
        w.write_all(b"avih")?;
        w.write_all(&56u32.to_le_bytes())?;
        w.write_all(&(1_000_000 / fps).to_le_bytes())?; // dwMicroSecPerFrame
        w.write_all(&0u32.to_le_bytes())?; // dwMaxBytesPerSec
        w.write_all(&0u32.to_le_bytes())?; // dwPaddingGranularity
        w.write_all(&0x10u32.to_le_bytes())?; // dwFlags = AVIF_HASINDEX
        let total_frames_pos = w.stream_position()?;
        w.write_all(&0u32.to_le_bytes())?; // dwTotalFrames (patched)
        w.write_all(&0u32.to_le_bytes())?; // dwInitialFrames
        w.write_all(&1u32.to_le_bytes())?; // dwStreams
        w.write_all(&0u32.to_le_bytes())?; // dwSuggestedBufferSize
        w.write_all(&width.to_le_bytes())?; // dwWidth
        w.write_all(&height.to_le_bytes())?; // dwHeight
        w.write_all(&[0u8; 16])?; // dwReserved[4]

        // strl LIST
        w.write_all(b"LIST")?;
        w.write_all(&strl_size.to_le_bytes())?;
        w.write_all(b"strl")?;

        // strh (AVIStreamHeader, 56 bytes)
        w.write_all(b"strh")?;
        w.write_all(&56u32.to_le_bytes())?;
        w.write_all(b"vids")?;
        w.write_all(b"MJPG")?;
        w.write_all(&0u32.to_le_bytes())?; // dwFlags
        w.write_all(&0u16.to_le_bytes())?; // wPriority
        w.write_all(&0u16.to_le_bytes())?; // wLanguage
        w.write_all(&0u32.to_le_bytes())?; // dwInitialFrames
        w.write_all(&1u32.to_le_bytes())?; // dwScale
        w.write_all(&fps.to_le_bytes())?; // dwRate (rate/scale = fps)
        w.write_all(&0u32.to_le_bytes())?; // dwStart
        let stream_len_pos = w.stream_position()?;
        w.write_all(&0u32.to_le_bytes())?; // dwLength (patched)
        w.write_all(&0u32.to_le_bytes())?; // dwSuggestedBufferSize
        w.write_all(&0xFFFF_FFFFu32.to_le_bytes())?; // dwQuality
        w.write_all(&0u32.to_le_bytes())?; // dwSampleSize
        w.write_all(&0i16.to_le_bytes())?; // rcFrame.left
        w.write_all(&0i16.to_le_bytes())?; // rcFrame.top
        w.write_all(&(width as i16).to_le_bytes())?; // rcFrame.right
        w.write_all(&(height as i16).to_le_bytes())?; // rcFrame.bottom

        // strf (BITMAPINFOHEADER, 40 bytes)
        w.write_all(b"strf")?;
        w.write_all(&40u32.to_le_bytes())?;
        w.write_all(&40u32.to_le_bytes())?; // biSize
        w.write_all(&width.to_le_bytes())?;
        w.write_all(&height.to_le_bytes())?;
        w.write_all(&1u16.to_le_bytes())?; // biPlanes
        w.write_all(&24u16.to_le_bytes())?; // biBitCount
        w.write_all(b"MJPG")?; // biCompression
        w.write_all(&width.saturating_mul(height).saturating_mul(3).to_le_bytes())?; // biSizeImage
        w.write_all(&0u32.to_le_bytes())?; // biXPelsPerMeter
        w.write_all(&0u32.to_le_bytes())?; // biYPelsPerMeter
        w.write_all(&0u32.to_le_bytes())?; // biClrUsed
        w.write_all(&0u32.to_le_bytes())?; // biClrImportant

        // --- movi ---
        w.write_all(b"LIST")?;
        let movi_size_pos = w.stream_position()?;
        w.write_all(&0u32.to_le_bytes())?; // movi LIST size (patched)
        let movi_fourcc_pos = w.stream_position()?;
        w.write_all(b"movi")?;

        Ok(Self {
            w,
            frames: Vec::new(),
            riff_size_pos,
            total_frames_pos,
            stream_len_pos,
            movi_size_pos,
            movi_fourcc_pos,
            movi_bytes: 4, // "movi"
        })
    }

    /// Append one JPEG frame as a `00dc` chunk (padded to an even byte boundary).
    pub fn write_frame(&mut self, jpeg: &[u8]) -> io::Result<()> {
        let chunk_pos = self.w.stream_position()?;
        let offset = (chunk_pos - self.movi_fourcc_pos) as u32; // relative to "movi"
        let size = jpeg.len() as u32;
        self.w.write_all(b"00dc")?;
        self.w.write_all(&size.to_le_bytes())?;
        self.w.write_all(jpeg)?;
        let mut chunk_total = 8 + size;
        if size % 2 == 1 {
            self.w.write_all(&[0])?; // pad to even
            chunk_total += 1;
        }
        self.frames.push((offset, size));
        self.movi_bytes += chunk_total;
        Ok(())
    }

    /// Number of frames written so far.
    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }

    /// Write the `idx1` index and patch all the placeholder sizes.
    pub fn finish(mut self) -> io::Result<()> {
        // idx1
        self.w.write_all(b"idx1")?;
        let idx_size = (self.frames.len() as u32) * 16;
        self.w.write_all(&idx_size.to_le_bytes())?;
        for (off, size) in &self.frames {
            self.w.write_all(b"00dc")?;
            self.w.write_all(&0x10u32.to_le_bytes())?; // AVIIF_KEYFRAME
            self.w.write_all(&off.to_le_bytes())?;
            self.w.write_all(&size.to_le_bytes())?;
        }
        let file_end = self.w.stream_position()?;
        let n = self.frames.len() as u32;

        self.w.seek(SeekFrom::Start(self.movi_size_pos))?;
        self.w.write_all(&self.movi_bytes.to_le_bytes())?;
        self.w.seek(SeekFrom::Start(self.total_frames_pos))?;
        self.w.write_all(&n.to_le_bytes())?;
        self.w.seek(SeekFrom::Start(self.stream_len_pos))?;
        self.w.write_all(&n.to_le_bytes())?;
        self.w.seek(SeekFrom::Start(self.riff_size_pos))?;
        self.w.write_all(&((file_end - 8) as u32).to_le_bytes())?;
        self.w.flush()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn le_u32(b: &[u8], pos: usize) -> u32 {
        u32::from_le_bytes([b[pos], b[pos + 1], b[pos + 2], b[pos + 3]])
    }
    fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
        haystack.windows(needle.len()).position(|w| w == needle)
    }

    #[test]
    fn writes_riff_avi_header() {
        let mut buf = Cursor::new(Vec::new());
        let w = AviWriter::new(&mut buf, 240, 320, 20).unwrap();
        w.finish().unwrap();
        let b = buf.into_inner();
        assert_eq!(&b[0..4], b"RIFF");
        assert_eq!(&b[8..12], b"AVI ");
        // RIFF size = file length - 8
        assert_eq!(le_u32(&b, 4) as usize, b.len() - 8);
    }

    #[test]
    fn frames_are_stored_verbatim_and_counted() {
        let mut buf = Cursor::new(Vec::new());
        let mut w = AviWriter::new(&mut buf, 4, 4, 25).unwrap();
        // two fake "jpeg" payloads (odd length forces padding on the first)
        w.write_frame(&[0xFF, 0xD8, 0xAA]).unwrap();
        w.write_frame(&[0xFF, 0xD8, 0xBB, 0xCC]).unwrap();
        assert_eq!(w.frame_count(), 2);
        w.finish().unwrap();
        let b = buf.into_inner();

        // movi + idx1 present, frame payload bytes present
        let movi = find(&b, b"movi").expect("movi");
        assert!(find(&b, b"idx1").is_some());
        assert!(find(&b, &[0xFF, 0xD8, 0xAA]).is_some());

        // dwTotalFrames patched to 2 (avih dwTotalFrames is at a fixed offset:
        // RIFF(12) + LIST/hdrl hdr(12) + avih hdr(8) + 4 fields*4 = 12+12+8+16)
        let total_frames = le_u32(&b, 12 + 12 + 8 + 16);
        assert_eq!(total_frames, 2);

        // first 00dc chunk sits right after "movi"; its idx1 offset must be 4
        let first_chunk = movi + 4;
        assert_eq!(&b[first_chunk..first_chunk + 4], b"00dc");
        let idx = find(&b, b"idx1").unwrap();
        // idx1: "idx1"(4) size(4) then entry: ckid(4) flags(4) offset(4) size(4)
        let first_off = le_u32(&b, idx + 8 + 8);
        assert_eq!(first_off, 4);
    }
}
