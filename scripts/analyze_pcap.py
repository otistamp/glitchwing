#!/usr/bin/env python3
"""Parse a PCAPdroid capture and summarize the DRCX5 control (UDP 8090) and
video (UDP 8080) traffic. Uses the IP header's total-length field to bound the
real payload, ignoring any PCAPdroid metadata trailer appended to each record."""
import struct, sys
from collections import Counter, OrderedDict

path = sys.argv[1]
with open(path, "rb") as f:
    data = f.read()

magic = data[:4]
if magic in (b"\xd4\xc3\xb2\xa1", b"\xa1\xb2\xc3\xd4"):
    le = magic == b"\xd4\xc3\xb2\xa1"
    end = "<" if le else ">"
    linktype = struct.unpack(end + "I", data[20:24])[0]
    off, rec_hdr = 24, 16
else:
    print("Unsupported pcap (maybe pcapng); magic:", magic.hex()); sys.exit(1)

def ip_start(frame):
    if frame and (frame[0] >> 4) == 4:
        return 0
    if len(frame) > 14 and (frame[14] >> 4) == 4:  # ethernet
        return 14
    return None

ctrl = Counter()          # 8-byte control payloads on :8090
ctrl_first = {}
video_sizes = Counter()
video_first_payload = None
video_pkts = 0
video_big_heads = []      # first bytes of large video chunks
video_small = []          # small 8080 packets (status/handshake)
ts_first = ts_last = None

n = 0
while off + rec_hdr <= len(data):
    ts_s, ts_us, caplen, origlen = struct.unpack(end + "IIII", data[off:off+rec_hdr])
    off += rec_hdr
    frame = data[off:off+caplen]
    off += caplen
    n += 1
    s = ip_start(frame)
    if s is None:
        continue
    ip = frame[s:]
    if len(ip) < 20:
        continue
    ihl = (ip[0] & 0x0F) * 4
    total = struct.unpack(">H", ip[2:4])[0]   # IP total length (real packet size)
    proto = ip[9]
    if proto != 17:   # UDP only
        continue
    src = ".".join(map(str, ip[12:16]))
    dst = ".".join(map(str, ip[16:20]))
    udp = ip[ihl:total]
    if len(udp) < 8:
        continue
    sport, dport, ulen = struct.unpack(">HHH", udp[0:6])
    payload = udp[8:ulen] if ulen >= 8 else udp[8:]
    ts = ts_s + ts_us / 1e6
    if ts_first is None:
        ts_first = ts
    ts_last = ts
    if dport == 8090 and len(payload) == 8:
        key = payload.hex()
        ctrl[key] += 1
        ctrl_first.setdefault(key, ts)
    elif (sport == 8080 or dport == 8080):
        if sport == 8080:  # drone -> phone = video frames
            video_pkts += 1
            video_sizes[len(payload)] += 1
            if video_first_payload is None and len(payload) > 4:
                video_first_payload = payload[:48]
            if len(payload) > 1000 and len(video_big_heads) < 8:
                video_big_heads.append((video_pkts, len(payload), payload[:32]))
            if len(payload) <= 16 and len(video_small) < 8:
                video_small.append((round(ts - ts_first, 1), payload))

print(f"records={n} duration={ts_last-ts_first:.1f}s\n")
print(f"=== CONTROL :8090 — {sum(ctrl.values())} pkts, {len(ctrl)} distinct payloads ===")
print("count  payload(hex)                  decoded [hdr roll pitch thr yaw flags csum ftr]")
for key, c in ctrl.most_common(40):
    b = bytes.fromhex(key)
    t0 = ctrl_first[key] - ts_first
    dec = f"hdr={b[0]:02x} roll={b[1]:3d} pit={b[2]:3d} thr={b[3]:3d} yaw={b[4]:3d} flags={b[5]:08b} csum={b[6]:02x} ftr={b[7]:02x}"
    print(f"{c:5d}  {' '.join(f'{x:02x}' for x in b)}   t0={t0:5.1f}s  {dec}")

print(f"\n=== VIDEO :8080 (drone->phone) — {video_pkts} pkts ===")
print("payload sizes (top):", video_sizes.most_common(8))
if video_first_payload:
    print("first payload bytes:", " ".join(f"{x:02x}" for x in video_first_payload))
print("\n-- large video chunk heads (pkt#, len, first 32 bytes) --")
for idx, ln, head in video_big_heads:
    print(f"  #{idx} len={ln}: {' '.join(f'{x:02x}' for x in head)}")
print("\n-- small :8080 packets (status/handshake) --")
for t, p in video_small:
    print(f"  t={t}s len={len(p)}: {' '.join(f'{x:02x}' for x in p)}")
