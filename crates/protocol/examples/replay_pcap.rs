//! Replay a PCAPdroid capture through the FrameReassembler to validate MJPEG
//! reassembly on real drone data, and dump the first few frames as .jpg.
//!
//! Usage: cargo run -p protocol --example replay_pcap -- <capture.pcap> <out_dir>

use protocol::FrameReassembler;
use std::fs;

fn u32le(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}
fn u16be(b: &[u8]) -> u16 {
    u16::from_be_bytes([b[0], b[1]])
}

fn main() {
    let mut args = std::env::args().skip(1);
    let pcap = args.next().expect("usage: replay_pcap <capture.pcap> <out_dir>");
    let out_dir = args.next().unwrap_or_else(|| "frames".into());
    fs::create_dir_all(&out_dir).unwrap();

    let data = fs::read(&pcap).unwrap();
    assert_eq!(&data[0..4], &[0xd4, 0xc3, 0xb2, 0xa1], "expected little-endian pcap");

    let mut reasm = FrameReassembler::new();
    let mut off = 24usize; // global header
    let (mut frames, mut valid, mut saved) = (0u32, 0u32, 0u32);

    while off + 16 <= data.len() {
        let caplen = u32le(&data[off + 8..off + 12]) as usize;
        off += 16;
        if off + caplen > data.len() {
            break;
        }
        let frame = &data[off..off + caplen];
        off += caplen;

        // Raw IPv4 either at 0 or after a 14-byte ethernet header.
        let s = if !frame.is_empty() && frame[0] >> 4 == 4 {
            0
        } else if frame.len() > 14 && frame[14] >> 4 == 4 {
            14
        } else {
            continue;
        };
        let ip = &frame[s..];
        if ip.len() < 20 || ip[9] != 17 {
            continue; // not UDP
        }
        let ihl = (ip[0] & 0x0f) as usize * 4;
        let total = u16be(&ip[2..4]) as usize;
        let src = &ip[12..16];
        if ip.len() < ihl + 8 || total > ip.len() {
            continue;
        }
        let udp = &ip[ihl..total];
        let sport = u16be(&udp[0..2]);
        let ulen = u16be(&udp[4..6]) as usize;
        if sport != 8080 || src != [192, 168, 4, 153] || ulen < 8 {
            continue;
        }
        let payload = &udp[8..ulen.min(udp.len())];

        if let Some(jpeg) = reasm.push(payload) {
            frames += 1;
            // Valid = JPEG SOI at start and an EOI marker present (trailing padding allowed).
            let ok = jpeg.starts_with(&[0xff, 0xd8])
                && jpeg.windows(2).any(|w| w == [0xff, 0xd9]);
            if ok {
                valid += 1;
            }
            if saved < 5 {
                let path = format!("{out_dir}/frame_{frames:03}.jpg");
                fs::write(&path, &jpeg).unwrap();
                println!("saved {path} ({} bytes, valid_jpeg={ok})", jpeg.len());
                saved += 1;
            }
        }
    }
    println!("\nreassembled {frames} frames, {valid} valid JPEGs (start FFD8 / end FFD9)");
}
