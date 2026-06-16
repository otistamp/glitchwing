//! Toy-drone wire protocol.
//!
//! Control: 8-byte UDP packets to `192.168.4.153:8090`.

pub mod avi;

/// Header byte of an active control packet.
pub const HEADER: u8 = 0x66;
/// Footer byte of an active control packet.
pub const FOOTER: u8 = 0x99;
/// Neutral/center value for roll, pitch, yaw — and throttle (altitude-hold drone).
pub const CENTER: u8 = 0x80;

// Flag bits (byte 5 of the control packet).
pub const FLAG_TAKEOFF: u8 = 0x01;
pub const FLAG_LAND: u8 = 0x02;
pub const FLAG_EMERGENCY: u8 = 0x04;
pub const FLAG_FLIP: u8 = 0x08;
pub const FLAG_HEADLESS: u8 = 0x10;
pub const FLAG_CALIBRATE: u8 = 0x80;

/// Current stick/flag state to send to the drone.
///
/// All axes are 0–255 with `CENTER` (128) as neutral. Throttle is also centered
/// (128 = hold altitude, >128 climb, <128 descend).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlState {
    pub roll: u8,
    pub pitch: u8,
    pub throttle: u8,
    pub yaw: u8,
    pub flags: u8,
}

impl ControlState {
    /// Neutral hover: all axes centered, no flags.
    pub fn neutral() -> Self {
        ControlState { roll: CENTER, pitch: CENTER, throttle: CENTER, yaw: CENTER, flags: 0 }
    }

    /// Encode to the 8-byte wire packet `66 roll pitch throttle yaw flags csum 99`.
    pub fn encode(&self) -> [u8; 8] {
        let csum = checksum(self.roll, self.pitch, self.throttle, self.yaw, self.flags);
        [HEADER, self.roll, self.pitch, self.throttle, self.yaw, self.flags, csum, FOOTER]
    }
}

/// Map a normalized axis value (`-1.0..=1.0`) to a centered protocol byte
/// (`0..=255`, with `0.0` → `CENTER` (128)). Out-of-range input is clamped.
pub fn axis_to_byte(v: f32) -> u8 {
    let clamped = v.clamp(-1.0, 1.0);
    ((clamped + 1.0) * 0.5 * 255.0).round() as u8
}

/// Apply an expo curve to a normalized axis (`-1.0..=1.0`). `amount` (0.0..=1.0)
/// blends between linear (0.0) and cubic (1.0): softer near center, full at the ends.
pub fn expo(v: f32, amount: f32) -> f32 {
    amount * v * v * v + (1.0 - amount) * v
}

/// Add a signed trim offset to an axis byte, saturating at 0..=255.
pub fn apply_trim(value: u8, trim: i8) -> u8 {
    (value as i32 + trim as i32).clamp(0, 255) as u8
}

/// Move `current` toward `target` by at most `max_step` (per call). Used to
/// rate-limit throttle so taps aren't abrupt.
pub fn ramp_toward(current: u8, target: u8, max_step: u8) -> u8 {
    if target > current {
        current.saturating_add(max_step).min(target)
    } else {
        current.saturating_sub(max_step).max(target)
    }
}

/// Checksum: XOR of the five payload bytes (roll, pitch, throttle, yaw, flags),
/// bumped by 1 if it collides with the header (`0x66`) or footer (`0x99`).
pub fn checksum(roll: u8, pitch: u8, throttle: u8, yaw: u8, flags: u8) -> u8 {
    let c = roll ^ pitch ^ throttle ^ yaw ^ flags;
    if c == HEADER || c == FOOTER {
        c + 1
    } else {
        c
    }
}

/// The idle/disarmed keepalive packet (`0xAA … 0x55`), sent when not actively commanding.
pub fn idle_keepalive() -> [u8; 8] {
    let csum = checksum(CENTER, CENTER, 0x00, CENTER, 0x00);
    [0xAA, CENTER, CENTER, 0x00, CENTER, 0x00, csum, 0x55]
}

// ---------------------------------------------------------------------------
// Video: MJPEG-over-UDP frame reassembly (drone -> phone, port 8080).
// Each datagram: [frameId][isFinal][chunkCount][0x00] + 4-byte vendor magic + JPEG slice.
// ---------------------------------------------------------------------------

/// Vendor magic in bytes 4–7 of every video chunk header.
pub const VIDEO_MAGIC: [u8; 4] = [0x54, 0x5A, 0x48, 0x01];

/// Parsed video chunk header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VideoChunkHeader {
    pub frame_id: u8,
    /// True for the last chunk of the frame.
    pub is_final: bool,
    /// Total chunk count for the frame (meaningful only when `is_final`).
    pub chunk_count: u8,
}

/// Parse a video datagram into its header and JPEG body slice.
/// Returns `None` if too short or the magic doesn't match.
pub fn parse_video_chunk(datagram: &[u8]) -> Option<(VideoChunkHeader, &[u8])> {
    if datagram.len() < 8 || datagram[4..8] != VIDEO_MAGIC {
        return None;
    }
    let header = VideoChunkHeader {
        frame_id: datagram[0],
        is_final: datagram[1] == 0x01,
        chunk_count: datagram[2],
    };
    Some((header, &datagram[8..]))
}

/// Reassembles complete JPEG frames from in-order video datagrams.
#[derive(Default)]
pub struct FrameReassembler {
    frame_id: Option<u8>,
    buf: Vec<u8>,
    received: u8,
}

impl FrameReassembler {
    pub fn new() -> Self {
        FrameReassembler::default()
    }

    /// Feed one UDP datagram. Returns the complete JPEG when a frame finishes;
    /// `None` while a frame is still arriving, or if it was dropped (lost chunk,
    /// new frame started early, or bad magic).
    pub fn push(&mut self, datagram: &[u8]) -> Option<Vec<u8>> {
        let (header, body) = parse_video_chunk(datagram)?;
        if self.frame_id != Some(header.frame_id) {
            self.frame_id = Some(header.frame_id);
            self.buf.clear();
            self.received = 0;
        }
        self.buf.extend_from_slice(body);
        self.received += 1;
        if header.is_final {
            let complete = self.received == header.chunk_count;
            let frame = std::mem::take(&mut self.buf);
            self.frame_id = None;
            self.received = 0;
            return complete.then_some(frame);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Reference vectors captured from the stock app (protocol spec §"Reference vectors").
    fn cs(roll: u8, pitch: u8, throttle: u8, yaw: u8, flags: u8) -> ControlState {
        ControlState { roll, pitch, throttle, yaw, flags }
    }

    #[test]
    fn neutral_hover_encodes_to_reference() {
        assert_eq!(ControlState::neutral().encode(), [0x66, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x99]);
    }

    #[test]
    fn climb_encodes_to_reference() {
        assert_eq!(cs(0x80, 0x80, 0x87, 0x80, 0x00).encode(), [0x66, 0x80, 0x80, 0x87, 0x80, 0x00, 0x07, 0x99]);
    }

    #[test]
    fn descend_with_yaw_encodes_to_reference() {
        assert_eq!(cs(0x80, 0x80, 0x54, 0x87, 0x00).encode(), [0x66, 0x80, 0x80, 0x54, 0x87, 0x00, 0xd3, 0x99]);
    }

    #[test]
    fn yaw_right_encodes_to_reference() {
        assert_eq!(cs(0x80, 0x80, 0x80, 0x82, 0x00).encode(), [0x66, 0x80, 0x80, 0x80, 0x82, 0x00, 0x02, 0x99]);
    }

    #[test]
    fn pitch_full_forward_encodes_to_reference() {
        assert_eq!(cs(0x8c, 0xff, 0x80, 0x80, 0x00).encode(), [0x66, 0x8c, 0xff, 0x80, 0x80, 0x00, 0x73, 0x99]);
    }

    #[test]
    fn roll_near_min_encodes_to_reference() {
        assert_eq!(cs(0x01, 0x80, 0x80, 0x80, 0x00).encode(), [0x66, 0x01, 0x80, 0x80, 0x80, 0x00, 0x81, 0x99]);
    }

    #[test]
    fn headless_flag_encodes_to_reference() {
        assert_eq!(cs(0x80, 0x80, 0x80, 0x80, FLAG_HEADLESS).encode(), [0x66, 0x80, 0x80, 0x80, 0x80, 0x10, 0x10, 0x99]);
    }

    #[test]
    fn calibrate_flag_encodes_to_reference() {
        assert_eq!(cs(0x80, 0x80, 0x80, 0x80, FLAG_CALIBRATE).encode(), [0x66, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x99]);
    }

    #[test]
    fn emergency_stop_encodes_to_reference() {
        assert_eq!(cs(0x82, 0x81, 0x80, 0x80, FLAG_EMERGENCY).encode(), [0x66, 0x82, 0x81, 0x80, 0x80, 0x04, 0x07, 0x99]);
    }

    #[test]
    fn idle_keepalive_matches_reference() {
        assert_eq!(idle_keepalive(), [0xaa, 0x80, 0x80, 0x00, 0x80, 0x00, 0x80, 0x55]);
    }

    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-4, "{a} != {b}");
    }

    #[test]
    fn expo_preserves_endpoints_and_center() {
        for amount in [0.0, 0.5, 1.0] {
            approx(expo(0.0, amount), 0.0);
            approx(expo(1.0, amount), 1.0);
            approx(expo(-1.0, amount), -1.0);
        }
    }

    #[test]
    fn expo_full_is_cubic_and_zero_is_linear() {
        approx(expo(0.5, 1.0), 0.125);
        approx(expo(0.5, 0.0), 0.5);
    }

    #[test]
    fn trim_offsets_and_saturates() {
        assert_eq!(apply_trim(128, 10), 138);
        assert_eq!(apply_trim(128, -10), 118);
        assert_eq!(apply_trim(250, 10), 255);
        assert_eq!(apply_trim(2, -10), 0);
    }

    #[test]
    fn ramp_moves_by_at_most_max_step() {
        assert_eq!(ramp_toward(128, 200, 16), 144);
        assert_eq!(ramp_toward(200, 128, 16), 184);
    }

    #[test]
    fn ramp_snaps_when_within_step() {
        assert_eq!(ramp_toward(128, 130, 16), 130);
        assert_eq!(ramp_toward(128, 128, 16), 128);
    }

    #[test]
    fn axis_center_is_128() {
        assert_eq!(axis_to_byte(0.0), 128);
    }

    #[test]
    fn axis_extremes_map_to_full_range() {
        assert_eq!(axis_to_byte(1.0), 255);
        assert_eq!(axis_to_byte(-1.0), 0);
    }

    #[test]
    fn axis_clamps_out_of_range() {
        assert_eq!(axis_to_byte(2.5), 255);
        assert_eq!(axis_to_byte(-2.5), 0);
    }

    #[test]
    fn axis_half_is_about_three_quarters() {
        assert_eq!(axis_to_byte(0.5), 191);
    }

    #[test]
    fn checksum_bumps_on_header_collision() {
        // XOR that lands on 0x66 must become 0x67 (0x66 XOR 0x00... -> use bytes that xor to 0x66).
        assert_eq!(checksum(0x66, 0x00, 0x00, 0x00, 0x00), 0x67);
    }

    #[test]
    fn checksum_bumps_on_footer_collision() {
        assert_eq!(checksum(0x99, 0x00, 0x00, 0x00, 0x00), 0x9a);
    }

    // --- video reassembly ---

    /// Build a video datagram with the real header layout.
    fn chunk(frame_id: u8, is_final: bool, count: u8, body: &[u8]) -> Vec<u8> {
        let b1 = if is_final { 0x01 } else { 0x00 };
        let b2 = if is_final { count } else { 0x00 };
        let mut v = vec![frame_id, b1, b2, 0x00, 0x54, 0x5A, 0x48, 0x01];
        v.extend_from_slice(body);
        v
    }

    #[test]
    fn parse_chunk_rejects_short_and_bad_magic() {
        assert!(parse_video_chunk(&[0x01, 0x00]).is_none());
        assert!(parse_video_chunk(&[1, 0, 0, 0, b'X', b'Y', b'Z', 1, 0xff]).is_none());
    }

    #[test]
    fn parse_chunk_reads_header_and_body() {
        let d = chunk(3, true, 5, &[0xff, 0xd9]);
        let (h, body) = parse_video_chunk(&d).expect("valid chunk");
        assert_eq!(h, VideoChunkHeader { frame_id: 3, is_final: true, chunk_count: 5 });
        assert_eq!(body, &[0xff, 0xd9]);
    }

    #[test]
    fn assembles_two_chunk_frame_in_order() {
        let mut r = FrameReassembler::new();
        assert_eq!(r.push(&chunk(1, false, 0, &[0xff, 0xd8, 0xaa])), None);
        let frame = r.push(&chunk(1, true, 2, &[0xbb, 0xff, 0xd9])).expect("frame complete");
        assert_eq!(frame, vec![0xff, 0xd8, 0xaa, 0xbb, 0xff, 0xd9]);
    }

    #[test]
    fn drops_frame_when_a_chunk_is_lost() {
        // Final chunk says count=3 but only 2 datagrams arrived -> incomplete, drop.
        let mut r = FrameReassembler::new();
        assert_eq!(r.push(&chunk(7, false, 0, &[0xff, 0xd8])), None);
        assert_eq!(r.push(&chunk(7, true, 3, &[0xff, 0xd9])), None);
    }

    #[test]
    fn new_frame_id_resets_incomplete_previous() {
        let mut r = FrameReassembler::new();
        r.push(&chunk(1, false, 0, &[0xff, 0xd8, 0x11])); // frame 1 never finishes
        // Frame 2 arrives complete in one (final) chunk.
        let frame = r.push(&chunk(2, true, 1, &[0xff, 0xd8, 0x22, 0xff, 0xd9])).expect("frame 2");
        assert_eq!(frame, vec![0xff, 0xd8, 0x22, 0xff, 0xd9]);
    }

    #[test]
    fn ignores_datagram_with_bad_magic() {
        let mut r = FrameReassembler::new();
        let mut bad = chunk(1, true, 1, &[0xff, 0xd8, 0xff, 0xd9]);
        bad[4] = b'X'; // corrupt magic
        assert_eq!(r.push(&bad), None);
    }
}
