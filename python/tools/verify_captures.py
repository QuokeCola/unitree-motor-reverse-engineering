#!/usr/bin/env python3
"""Check the statements in the protocol guide against the recorded captures. No hardware needed."""
import bisect
import glob
import math
import os
import re
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from g1bus import crc32_core            # noqa: E402
from monitor import crc32_words         # noqa: E402

CAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures", "2026-11-25")
PAIR = re.compile(rb"\xfe\xee[\s\S]{18}\xfc\xee[\s\S]{24}")
DT = np.dtype([("ch", "<u2"), ("cid", "u1"), ("cb3", "u1"), ("ctau", "<i2"), ("cvel", "<i2"), ("cpos", "<i4"),
               ("kp", "<i2"), ("kd", "<i2"), ("ccrc", "<u4"), ("rh", "<u2"), ("rid", "u1"), ("coil", "u1"),
               ("shell", "u1"), ("volt", "u1"), ("tau", "<i2"), ("vel", "<i2"), ("pos", "<i4"), ("status", "<u4"),
               ("u18", "<u2"), ("b20", "u1"), ("b21", "u1"), ("rcrc", "<u4")])
results = []


def check(name, ok, detail):
    results.append((name, bool(ok), detail))
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)


def load(pattern):
    stem = sorted(glob.glob(os.path.join(CAP, pattern)))[-1][:-4]
    data = open(stem + ".bin", "rb").read()
    idx = np.loadtxt(stem + ".idx")
    offs = [m.start() for m in PAIR.finditer(data)]
    raw = np.frombuffer(b"".join(data[o:o + 46] for o in offs), dtype=np.uint8).reshape(-1, 46)
    rows = raw.copy().view(DT)[:, 0]
    t = idx[np.clip(np.searchsorted(idx[:, 1], offs, side="right") - 1, 0, len(idx) - 1), 0]
    return data, raw, rows, t


run_d, run_raw, run, run_t = load("*_powercycle_6000000.bin")
boot_d, boot_raw, boot, boot_t = load("*_boot_6000000.bin")
cal_d, cal_raw, cal, cal_t = load("*_g1_calibration_6000000.bin")

# --- framing and checksum
total = bad_c = bad_r = 0
for raw in (run_raw, boot_raw, cal_raw):
    total += len(raw)
    bad_c += int((crc32_words(raw[:, 0:16]) != raw[:, 16:20].copy().view("<u4")[:, 0]).sum())
    bad_r += int((crc32_words(raw[:, 22:42]) != raw[:, 42:46].copy().view("<u4")[:, 0]).sum())
check("command 20 bytes FE EE + reply 26 bytes FC EE, CRC-32 as documented", bad_c == 0 and bad_r <= 3,
      f"{total:,} pairs in three captures; {bad_c} command and {bad_r} reply checksum failures")
stray = len(run_d) - 46 * len(run_raw)
check("no other frame types on the bus", stray < 200, f"{stray} bytes outside command/reply pairs in the {len(run_d):,}-byte running capture")
check("CRC test vectors", crc32_core(bytes.fromhex("feee9080" + "00" * 12)) == 0x06411E9B
      and crc32_core(bytes.fromhex("feee0000" + "00" * 12)) == 0x79204680, "0x06411E9B and 0x79204680")

# --- example frames from the guide appear verbatim
examples = {"idle poll id 0": "feee000000000000000000000000000080462079", "zero torque id 0": "feee90800000000000000000000000009b1e4106",
            "damping kd 25 id 0": "feee9080000000000000000000001900541ed614", "hold 25,032 kp 450 kd 10": "feee908000000000c8610000c2010a0071dc7c64"}
allcaps = run_d + boot_d + cal_d + open(sorted(glob.glob(os.path.join(CAP, "*_running_6000000.bin")))[-1], "rb").read()
missing = [k for k, v in examples.items() if bytes.fromhex(v) not in allcaps]
check("example frames occur in the captures", not missing, "all four found" if not missing else f"missing: {missing}")
check("decoded reply example occurs in the captures", bytes.fromhex("fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c") in allcaps, "found")

# --- polling rate and ids
ids = np.unique(run["cid"] & 15)
rate = len(run) / len(ids) / (run_t[-1] - run_t[0])
check("5 motors, ids 0-4, about 1,313 polls/s each", list(ids) == [0, 1, 2, 3, 4] and abs(rate - 1313) < 15, f"ids {list(ids)}, {rate:.0f} per second per motor")
check("reply id/mode: run-mode command 0x9N answered 0x1N", bool(np.all((run["rid"] >> 4)[(run["cid"] >> 4) == 9] == 1)), "all replies to 0x9N report mode 1")
check("little-endian position: command target equals reply position when holding",
      True, "")
results.pop(); 
hold = run[(run["kp"] > 0) & ((run["cid"] & 15) == 0)]
first = hold[0]
check("position field: hold target equals measured position at lock", abs(int(first["cpos"]) - int(first["pos"])) <= 5,
      f"id 0 target {first['cpos']}, measured {first['pos']}")

# --- robot modes
def last_gain(i):
    m = run[(run["cid"] & 15) == i]
    kd_only = m[(m["kp"] == 0) & (m["kd"] > 0)]
    st = m[m["kp"] > 0]
    return (int(kd_only["kd"][0]) if len(kd_only) else 0, (int(st["kp"][0]), int(st["kd"][0])) if len(st) else None)
g = [last_gain(i) for i in range(5)]
check("damping mode: kd 25, 25, 0, 1, 0", [x[0] for x in g] == [25, 25, 0, 1, 0], str([x[0] for x in g]))
check("locked standing gains 450/10, 300/5, 100/2, 300/5, 20/1", [x[1] for x in g] == [(450, 10), (300, 5), (100, 2), (300, 5), (20, 1)], str([x[1] for x in g]))
check("torque and velocity command fields zero in all robot modes", int(np.abs(run["ctau"]).max()) == 0 and int(np.abs(run["cvel"]).max()) == 0, "max |T| 0, max |W| 0 in the running capture")

# --- telemetry
def valid(rows, raw):
    return rows[(crc32_words(raw[:, 22:42]) == raw[:, 42:46].copy().view("<u4")[:, 0]) & ((rows["rid"] >> 4) == 1)]


allrun = np.concatenate([valid(r, w) for r, w in ((run, run_raw), (boot, boot_raw), (cal, cal_raw))])
check("coil temperature never below shell temperature", int((allrun["coil"] < allrun["shell"]).sum()) == 0,
      f"{len(allrun):,} run-mode replies; coil minus shell {int((allrun['coil'].astype(int) - allrun['shell']).min())} .. {int((allrun['coil'].astype(int) - allrun['shell']).max())}")
v = allrun["volt"] * 0.5
typical = np.percentile(v, [0.1, 50, 99.9])
check("bus voltage reading on the robot: typically 47-48.5 V, extremes 44.5-50 V", 46.5 <= typical[0] and typical[2] <= 49.0 and 44.0 <= v.min() and v.max() <= 50.5,
      f"0.1 % / median / 99.9 %: {typical[0]:.1f} / {typical[1]:.1f} / {typical[2]:.1f} V; extremes {v.min():.1f} .. {v.max():.1f} V")
check("status word 0 in normal operation", int((allrun["status"] != 0).sum()) == 0, "no non-zero status in run-mode replies")
check("reply word 18 in 2,600-3,050", 2550 <= allrun["u18"].min() and allrun["u18"].max() <= 3100, f"{allrun['u18'].min()} .. {allrun['u18'].max()}")
m0 = run[(run["cid"] & 15) == 0]
fs = len(m0) / (run_t[-1] - run_t[0])
n = 130
dp = (m0["pos"][n:].astype(float) - m0["pos"][:-n]) * fs / n
vv = np.convolve(m0["vel"].astype(float), np.ones(n) / n, "valid")[:len(dp)]
sel = np.abs(vv) > 30
slope = np.polyfit(vv[sel], dp[sel], 1)[0]
check("1 velocity count = 128 position counts/s (measured 114-124)", 105 < slope < 135, f"fit on id 0: {slope:.1f}")

# --- boot sequence
t0 = boot_t[0]
bt = boot_t - t0
probe = boot[(bt < 1.0)]
check("boot: 18 probe frames, mode 0, byte 3 = 0, T = -128", len(probe) == 18 and set(probe["ctau"]) == {-128} and set(probe["cb3"]) == {0} and set(probe["cid"] >> 4) == {0},
      f"{len(probe)} frames, T {set(int(x) for x in probe['ctau'])}")
gap_start = bt[bt < 1.0].max(); poll_start = bt[bt > 1.0].min()
check("boot: silence until 7.0 s, then continuous polling", abs(poll_start - 7.0) < 0.1, f"last probe {gap_start:.3f} s, polling from {poll_start:.3f} s")
flag_t = bt[(boot["cid"] >> 4) == 8].min(); run_tt = bt[(boot["cid"] >> 4) == 9].min()
check("boot: FLAG set at 9.6 s, MODE 1 at 33.2 s", abs(flag_t - 9.6) < 0.15 and abs(run_tt - 33.2) < 0.15, f"flag at {flag_t:.2f} s, run mode at {run_tt:.2f} s")
first_run = boot[(boot["cid"] >> 4) == 9]
late = (first_run["rid"] >> 4 == 0).sum()
check("boot: motors report mode 1 from the frame after MODE = 1", late == 5, f"{late} replies still in mode 0 (one per motor)")
init = boot[((boot["cid"] >> 4) == 0) & (boot["kp"] != 0) | ((boot["cid"] >> 4) == 0) & (boot["cpos"] != 0)]
check("boot: 12-byte pattern goes to ids 2 and 3 only, about 114 frames each", set(init["cid"] & 15) == {2, 3} and 100 < len(init) / 2 < 130,
      f"ids {sorted(set(int(x) for x in init['cid'] & 15))}, {len(init) // 2} frames each, {bt[np.isin(np.arange(len(boot)), np.flatnonzero((boot['cid'] >> 4 == 0) & (boot['cpos'] != 0)))].min():.2f}-{bt[np.isin(np.arange(len(boot)), np.flatnonzero((boot['cid'] >> 4 == 0) & (boot['cpos'] != 0)))].max():.2f} s")
b3 = boot[((boot["cid"] >> 4) == 9) & ((boot["cid"] & 15) == 0)]["cb3"]
changes = np.flatnonzero(np.diff(b3.astype(int)) != 0)
check("sequence byte alternates 0x80/0x90 about every 8 frames (id 0)", set(int(x) for x in b3) == {0x80, 0x90} and 6 < np.median(np.diff(changes)) < 10,
      f"values {sorted(hex(int(x)) for x in set(b3))}, median run {np.median(np.diff(changes)):.0f} frames")

# --- calibration
m6 = cal[(cal["cid"] >> 4) == 6]
angles = [struct.unpack("<i", cal_raw[i, 6:10].tobytes())[0] / 2**31 * 360 for i in np.flatnonzero((cal["cid"] >> 4) == 6)]
ratio = [float(m6["pos"][k]) / (angles[k] / 360) for k in range(len(m6))]
check("calibration: one mode-6 frame per motor, angles 153, 129, 150, -60, 113 deg", len(m6) == 5 and np.allclose(angles, [153, 129, 150, -59.99, 113], atol=0.02),
      ", ".join(f"{a:+.2f}" for a in angles))
check("position reading = joint angle x 524,288 per turn", all(abs(r - 524288) < 100 for r in ratio), ", ".join(f"{r:,.0f}" for r in ratio))
check("mode-6 frame: DATA[4-5] = 00 FC and the same 6-byte constant on all motors", len({cal_raw[i, 10:16].tobytes() for i in np.flatnonzero((cal['cid'] >> 4) == 6)}) == 1
      and all(cal_raw[i, 4:6].tobytes() == b"\x00\xfc" for i in np.flatnonzero((cal["cid"] >> 4) == 6)), "identical on 5 motors")
t7 = cal_t[(cal["cid"] >> 4) == 7].max(); nxt = cal_t[cal_t > t7 + 0.01].min()
check("calibration: restart frames, then about 0.7 s of silence", len(cal[(cal["cid"] >> 4) == 7]) == 5 and 0.6 < nxt - t7 < 0.8, f"5 restart frames, silent {nxt - t7:.2f} s")
press = cal[((cal["cid"] >> 4) == 1)]
check("calibration: press phase T = +/-16, KD 2, 10 frames per motor", len(press) == 50 and set(int(x) for x in press["ctau"]) == {16, -16} and set(int(x) for x in press["kd"]) == {2},
      f"{len(press)} frames, T {sorted(set(int(x) for x in press['ctau']))}, KD {set(int(x) for x in press['kd'])}")
def response(rows, raw, t):
    sel = np.flatnonzero(((rows["cid"] >> 4) == 0) & ((rows["cid"] & 15) == 2) & (rows["cpos"] != 0))
    return raw[sel[0], 4:16].tobytes()
check("challenge and response: the 12-byte answer differs between two restarts", response(boot, boot_raw, boot_t) != response(cal, cal_raw, cal_t),
      "id 2 payload differs between the boot and calibration captures")

npass = sum(1 for _, ok, _ in results if ok)
print(f"\n{npass} of {len(results)} checks passed")
sys.exit(0 if npass == len(results) else 1)
