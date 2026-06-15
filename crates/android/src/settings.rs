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
}

pub const ACTIONS: [Action; 8] = [
    Action::Arm,
    Action::Takeoff,
    Action::Land,
    Action::Flip,
    Action::Calibrate,
    Action::Headless,
    Action::Emergency,
    Action::TrimReset,
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
            Action::Emergency => "EMERGENCY",
            Action::TrimReset => "TRIM RESET",
        }
    }
}

/// Action → gamepad keycode (indexed by `Action as usize`), plus config flags.
#[derive(Clone, Copy)]
pub struct Bindings {
    pub keys: [u32; 8],
    /// Throttle from L2/R2 triggers (true) vs the left-stick Y axis (false).
    pub throttle_triggers: bool,
}

impl Default for Bindings {
    fn default() -> Self {
        // Start, B, A, Y, X, L1, Select, R1
        Bindings { keys: [108, 97, 96, 100, 99, 102, 109, 103], throttle_triggers: false }
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
        if nums.len() >= 8 {
            b.keys.copy_from_slice(&nums[..8]);
            b.throttle_triggers = nums.get(8).is_some_and(|&v| v != 0);
        }
    }
    b
}

pub fn save(path: &str, b: &Bindings) {
    let mut nums: Vec<u32> = b.keys.to_vec();
    nums.push(b.throttle_triggers as u32);
    let s = nums.iter().map(|k| k.to_string()).collect::<Vec<_>>().join(" ");
    let _ = fs::write(path, s);
}
