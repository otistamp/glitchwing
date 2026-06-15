//! Gamepad key bindings + persistence (Android).

use std::fs;

/// Remappable flight actions.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Action {
    Arm = 0,
    Takeoff = 1,
    Land = 2,
    Flip = 3,
    Calibrate = 4,
    Headless = 5,
    Emergency = 6,
    TrimReset = 7,
    /// Button that cycles the speed/rate preset (LOW -> MED -> HIGH).
    Speed = 8,
    /// Held modifier: while down, the D-pad left/right trims yaw instead of roll.
    YawTrim = 9,
}

pub const ACTIONS: [Action; 10] = [
    Action::Arm,
    Action::Takeoff,
    Action::Land,
    Action::Flip,
    Action::Calibrate,
    Action::Headless,
    Action::Emergency,
    Action::TrimReset,
    Action::YawTrim,
    Action::Speed,
];

impl Action {
    pub fn label(self) -> &'static str {
        match self {
            Action::Arm => "ARM",
            Action::Takeoff => "TAKEOFF",
            Action::Land => "LAND",
            Action::Flip => "FLIP",
            Action::Calibrate => "CALIBRATE",
            Action::Headless => "HEADLESS",
            Action::Emergency => "KILLSWITCH",
            Action::TrimReset => "TRIM RESET",
            Action::Speed => "SPEED",
            Action::YawTrim => "YAW TRIM",
        }
    }
}

/// Max stick deflection (fraction of full) for each speed preset: LOW, MED, HIGH.
/// Mirrors the stock app's three speed tiers (slow/medium/full-rate).
pub const SPEED_DEFLECTION: [f32; 3] = [0.35, 0.6, 0.9];

/// Short HUD label for a speed level.
pub fn speed_name(level: u8) -> &'static str {
    match level {
        0 => "LO",
        1 => "MD",
        _ => "HI",
    }
}

/// Action → gamepad keycode (indexed by `Action as usize`), plus config flags.
#[derive(Clone, Copy)]
pub struct Bindings {
    pub keys: [u32; 10],
    /// Throttle from L2/R2 triggers (true) vs the left-stick Y axis (false).
    pub throttle_triggers: bool,
    /// Speed/rate preset: 0 = LOW, 1 = MED, 2 = HIGH.
    pub speed: u8,
}

impl Default for Bindings {
    fn default() -> Self {
        // Start, B, A, Y, X, L1, Select, R1, Mode, R2  (indexed by Action; YawTrim
        // on R2 — a trigger you deliberately hold, not an easily-nudged stick click)
        Bindings { keys: [108, 97, 96, 100, 99, 102, 109, 103, 110, 105], throttle_triggers: false, speed: 1 }
    }
}

impl Bindings {
    pub fn get(&self, a: Action) -> u32 {
        self.keys[a as usize]
    }
    pub fn set(&mut self, a: Action, kc: u32) {
        self.keys[a as usize] = kc;
    }
}

/// Friendly label for a gamepad keycode.
pub fn button_name(kc: u32) -> String {
    let s = match kc {
        96 => "A",
        97 => "B",
        99 => "X",
        100 => "Y",
        102 => "L1",
        103 => "R1",
        104 => "L2",
        105 => "R2",
        106 => "L-STICK",
        107 => "R-STICK",
        108 => "START",
        109 => "SELECT",
        110 => "MODE",
        _ => return format!("K{kc}"),
    };
    s.to_string()
}

pub fn load(path: &str) -> Bindings {
    let mut b = Bindings::default();
    if let Ok(s) = fs::read_to_string(path) {
        let nums: Vec<u32> = s.split_whitespace().filter_map(|t| t.parse().ok()).collect();
        if nums.len() >= 12 {
            // Current format: 10 keys + throttle flag + speed level.
            b.keys.copy_from_slice(&nums[..10]);
            b.throttle_triggers = nums[10] != 0;
            b.speed = (nums[11] as u8).min(2);
        } else if nums.len() >= 11 {
            // Prior format: 9 keys + throttle + speed (YawTrim binding stays default).
            b.keys[..9].copy_from_slice(&nums[..9]);
            b.throttle_triggers = nums[9] != 0;
            b.speed = (nums[10] as u8).min(2);
        } else if nums.len() >= 8 {
            // Legacy format: 8 keys (+ optional throttle flag). Keep the new Speed
            // and YawTrim bindings + level at their defaults.
            b.keys[..8].copy_from_slice(&nums[..8]);
            b.throttle_triggers = nums.get(8).is_some_and(|&v| v != 0);
        }
    }
    b
}

pub fn save(path: &str, b: &Bindings) {
    let mut nums: Vec<u32> = b.keys.to_vec();
    nums.push(b.throttle_triggers as u32);
    nums.push(b.speed as u32);
    let s = nums.iter().map(|k| k.to_string()).collect::<Vec<_>>().join(" ");
    let _ = fs::write(path, s);
}
