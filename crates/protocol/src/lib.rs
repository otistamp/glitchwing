//! DRCX5 / the stock app wire protocol.
//!
//! Control: 8-byte UDP packets to `192.168.4.153:8090`. See
//! `docs/superpowers/specs/2026-06-13-drcx5-protocol-spec.md`.

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

    #[test]
    fn checksum_bumps_on_header_collision() {
        // XOR that lands on 0x66 must become 0x67 (0x66 XOR 0x00... -> use bytes that xor to 0x66).
        assert_eq!(checksum(0x66, 0x00, 0x00, 0x00, 0x00), 0x67);
    }

    #[test]
    fn checksum_bumps_on_footer_collision() {
        assert_eq!(checksum(0x99, 0x00, 0x00, 0x00, 0x00), 0x9a);
    }
}
