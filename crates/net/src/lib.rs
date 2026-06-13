//! DRCX5 UDP link.
//!
//! - Control: a fixed-rate sender to `192.168.4.153:8090` whose output is decided
//!   by [`command_packet`] — the self-neutralizing failsafe lives there.
//! - Video: request + receive the MJPEG stream from `:8080`, reassembled into frames.

use std::net::{SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use protocol::{idle_keepalive, ControlState, FrameReassembler, FLAG_LAND};

/// Default drone address on its WiFi AP.
pub const DRONE_IP: [u8; 4] = [192, 168, 4, 153];
pub const CONTROL_PORT: u16 = 8090;
pub const VIDEO_PORT: u16 = 8080;

/// Two-byte request that starts the video stream (`'B' 0x76`).
pub const VIDEO_START: [u8; 2] = [0x42, 0x76];

/// How long without a control update before the sender forces the failsafe.
pub const STALE_AFTER: Duration = Duration::from_millis(200);
/// Control send interval (~25 Hz).
pub const SEND_INTERVAL: Duration = Duration::from_millis(40);

/// The packet sent when control input has gone stale (lost gamepad / dead input
/// thread / hung UI): neutral sticks + the one-key **land** flag for a controlled descent.
pub fn failsafe_packet() -> [u8; 8] {
    ControlState { flags: FLAG_LAND, ..ControlState::neutral() }.encode()
}

/// Decide which 8-byte packet the sender should transmit this cycle.
///
/// - disarmed → idle keepalive (`0xAA…0x55`)
/// - armed + fresh input → the current control state
/// - armed + stale input (`since_update >= stale_after`) → [`failsafe_packet`]
pub fn command_packet(
    armed: bool,
    state: ControlState,
    since_update: Duration,
    stale_after: Duration,
) -> [u8; 8] {
    if !armed {
        return idle_keepalive();
    }
    if since_update >= stale_after {
        return failsafe_packet();
    }
    state.encode()
}

// ---------------------------------------------------------------------------
// Runtime: threaded UDP link to the drone.
// ---------------------------------------------------------------------------

/// Addresses and timing for a [`DroneLink`].
#[derive(Clone, Copy, Debug)]
pub struct LinkConfig {
    pub control_addr: SocketAddr,
    pub video_addr: SocketAddr,
    pub send_interval: Duration,
    pub stale_after: Duration,
}

impl Default for LinkConfig {
    fn default() -> Self {
        let ip = std::net::Ipv4Addr::from(DRONE_IP);
        LinkConfig {
            control_addr: SocketAddr::V4(SocketAddrV4::new(ip, CONTROL_PORT)),
            video_addr: SocketAddr::V4(SocketAddrV4::new(ip, VIDEO_PORT)),
            send_interval: SEND_INTERVAL,
            stale_after: STALE_AFTER,
        }
    }
}

struct Inner {
    state: ControlState,
    armed: bool,
    updated: Instant,
}

/// Cheap, cloneable handle for updating control state. Every update refreshes the
/// freshness timestamp the sender's staleness failsafe watches.
#[derive(Clone)]
pub struct ControlHandle {
    inner: Arc<Mutex<Inner>>,
}

impl ControlHandle {
    /// Set the stick/flag state to transmit and mark input as fresh.
    pub fn set(&self, state: ControlState) {
        let mut g = self.inner.lock().unwrap();
        g.state = state;
        g.updated = Instant::now();
    }

    /// Arm (allow active commands). Also counts as a fresh input.
    pub fn arm(&self) {
        let mut g = self.inner.lock().unwrap();
        g.armed = true;
        g.updated = Instant::now();
    }

    /// Disarm — sender reverts to the idle keepalive.
    pub fn disarm(&self) {
        self.inner.lock().unwrap().armed = false;
    }
}

/// A running link: a fixed-rate control sender and a video receiver, each on its
/// own thread. Drop or call [`DroneLink::stop`] to shut them down.
pub struct DroneLink {
    pub control: ControlHandle,
    pub frames: Receiver<Vec<u8>>,
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
}

impl DroneLink {
    /// Bind sockets and spawn the sender + video threads.
    pub fn start(cfg: LinkConfig) -> std::io::Result<DroneLink> {
        let inner = Arc::new(Mutex::new(Inner {
            state: ControlState::neutral(),
            armed: false,
            updated: Instant::now(),
        }));
        let stop = Arc::new(AtomicBool::new(false));

        // Control sender.
        let ctrl_sock = UdpSocket::bind(("0.0.0.0", 0))?;
        let send_thread = {
            let inner = inner.clone();
            let stop = stop.clone();
            std::thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    let pkt = {
                        let g = inner.lock().unwrap();
                        command_packet(g.armed, g.state, g.updated.elapsed(), cfg.stale_after)
                    };
                    let _ = ctrl_sock.send_to(&pkt, cfg.control_addr);
                    std::thread::sleep(cfg.send_interval);
                }
            })
        };

        // Video request + receive.
        let video_sock = UdpSocket::bind(("0.0.0.0", 0))?;
        video_sock.set_read_timeout(Some(Duration::from_millis(200)))?;
        let _ = video_sock.send_to(&VIDEO_START, cfg.video_addr);
        let (frames_tx, frames) = mpsc::channel();
        let video_thread = {
            let stop = stop.clone();
            std::thread::spawn(move || {
                let mut reasm = FrameReassembler::new();
                let mut buf = [0u8; 2048];
                let mut last_req = Instant::now();
                while !stop.load(Ordering::Relaxed) {
                    // Re-request the stream periodically as a keepalive.
                    if last_req.elapsed() >= Duration::from_secs(1) {
                        let _ = video_sock.send_to(&VIDEO_START, cfg.video_addr);
                        last_req = Instant::now();
                    }
                    match video_sock.recv(&mut buf) {
                        Ok(n) => {
                            if let Some(frame) = reasm.push(&buf[..n]) {
                                if frames_tx.send(frame).is_err() {
                                    break; // receiver dropped
                                }
                            }
                        }
                        Err(_) => {} // timeout / would-block: loop to check stop flag
                    }
                }
            })
        };

        Ok(DroneLink {
            control: ControlHandle { inner },
            frames,
            stop,
            threads: vec![send_thread, video_thread],
        })
    }

    /// Signal both threads to stop and join them.
    pub fn stop(self) {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads {
            let _ = t.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::FLAG_LAND;

    fn climb() -> ControlState {
        ControlState { throttle: 0x87, ..ControlState::neutral() }
    }

    #[test]
    fn disarmed_sends_idle_keepalive() {
        assert_eq!(
            command_packet(false, climb(), Duration::ZERO, STALE_AFTER),
            idle_keepalive()
        );
    }

    #[test]
    fn armed_and_fresh_sends_current_state() {
        assert_eq!(
            command_packet(true, climb(), Duration::from_millis(10), STALE_AFTER),
            climb().encode()
        );
    }

    #[test]
    fn armed_but_stale_sends_failsafe_land() {
        let pkt = command_packet(true, climb(), Duration::from_millis(300), STALE_AFTER);
        assert_eq!(pkt, failsafe_packet());
        assert_ne!(pkt, climb().encode(), "must not keep commanding climb when stale");
        assert_eq!(pkt[5] & FLAG_LAND, FLAG_LAND, "failsafe must set the land flag");
    }

    #[test]
    fn staleness_boundary_is_inclusive() {
        // Exactly at the threshold counts as stale.
        let pkt = command_packet(true, climb(), STALE_AFTER, STALE_AFTER);
        assert_eq!(pkt, failsafe_packet());
    }

    // --- loopback integration: a fake drone socket on localhost ---

    fn loopback() -> UdpSocket {
        UdpSocket::bind(("127.0.0.1", 0)).unwrap()
    }

    /// Read up to `budget` datagrams; return true if any equals `want`.
    fn read_until(sock: &UdpSocket, want: [u8; 8], budget: usize) -> bool {
        let mut buf = [0u8; 32];
        for _ in 0..budget {
            if let Ok(n) = sock.recv(&mut buf) {
                if n == 8 && buf[..8] == want {
                    return true;
                }
            }
        }
        false
    }

    #[test]
    fn sender_emits_idle_then_active_then_failsafe() {
        let drone = loopback();
        drone.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
        let video_sink = loopback();
        let cfg = LinkConfig {
            control_addr: drone.local_addr().unwrap(),
            video_addr: video_sink.local_addr().unwrap(),
            send_interval: Duration::from_millis(10),
            stale_after: Duration::from_millis(150),
        };
        let link = DroneLink::start(cfg).unwrap();

        // Disarmed → idle keepalive.
        assert!(read_until(&drone, idle_keepalive(), 5), "expected idle keepalive when disarmed");

        // Armed + fresh climb input → climb packet (catch within the freshness window).
        link.control.arm();
        link.control.set(climb());
        assert!(read_until(&drone, climb().encode(), 40), "expected active climb packet");

        // Stop refreshing input → after the stale window, failsafe land. Budget must
        // exceed the queued-climb backlog accumulated during the sleep; queued reads
        // are instant and failsafe packets then keep arriving.
        std::thread::sleep(Duration::from_millis(200));
        assert!(read_until(&drone, failsafe_packet(), 100), "expected failsafe land when stale");

        link.stop();
    }

    #[test]
    fn video_receiver_reassembles_frame_from_fake_drone() {
        let drone = loopback();
        drone.set_read_timeout(Some(Duration::from_secs(1))).unwrap();
        let ctrl_sink = loopback();
        let cfg = LinkConfig {
            control_addr: ctrl_sink.local_addr().unwrap(),
            video_addr: drone.local_addr().unwrap(),
            send_interval: Duration::from_millis(50),
            stale_after: Duration::from_millis(200),
        };
        let link = DroneLink::start(cfg).unwrap();

        // The link sends VIDEO_START; learn its source address.
        let mut buf = [0u8; 64];
        let (n, src) = drone.recv_from(&mut buf).expect("VIDEO_START request");
        assert_eq!(&buf[..n], &VIDEO_START);

        // Reply with a complete single-chunk frame (final, count=1).
        let body = [0xff, 0xd8, 0x11, 0x22, 0xff, 0xd9];
        let mut chunk = vec![0x01, 0x01, 0x01, 0x00, 0x54, 0x5a, 0x48, 0x01];
        chunk.extend_from_slice(&body);
        drone.send_to(&chunk, src).unwrap();

        let frame = link
            .frames
            .recv_timeout(Duration::from_secs(2))
            .expect("a reassembled frame");
        assert_eq!(frame, body);

        link.stop();
    }
}
