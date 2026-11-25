#!/usr/bin/env python3
"""Live check of the protocol guide's statements on the bench wrist (ids 4, 5, 6, no controller).

Moves the joints. Needs calibration/limits.json from a hand sweep of the end stops.
  verify_bench.py            everything except the deliberate fault
  verify_bench.py --fault    also latch fault 0x8 on purpose with a hard brake and clear it with mode 7
"""
import argparse
import csv
import glob
import math
import os
import statistics
import subprocess
import sys
import time

from g1_joint_motor_sdk import MODE_RESTART, MODE_RUN, MotorBus, MotorCmd, load_joints

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
PY = os.path.join(ROOT, ".venv", "bin", "python")
LOGDIR = os.path.join(ROOT, "captures", time.strftime("%Y-%m-%d"))
IDS = [4, 5, 6]
KD = {4: 40, 5: 80, 6: 80}
T = 4                     # the motor used for single-motor tests
results = []

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--fault", action="store_true")
ap.add_argument("--skip-motion-runs", action="store_true", help="skip the sine runs through sweep.py")
args = ap.parse_args()
os.makedirs(LOGDIR, exist_ok=True)


def check(name, ok, detail):
    results.append((name, ok, detail))
    print(f"{'PASS' if ok else 'FAIL' if ok is False else 'INFO'}  {name}: {detail}", flush=True)


class Rig:
    def __init__(self):
        self.bus = MotorBus()
        self.joints = load_joints(os.path.join(ROOT, "calibration", "limits.json"))
        for i, j in self.joints.items():
            self.bus.set_joint(i, j)
        self.n = 0

    def raw(self, motor_id, mode=0, flag=None, **f):
        self.n += 1
        flag = (mode != 0) if flag is None else flag
        b3 = (0x80 if (self.n // 8) % 2 == 0 else 0x90) if flag else 0
        return self.bus.transfer(MotorCmd(id=motor_id, mode=mode, flag=flag, byte3=b3, **f))

    def rest(self, motor_id):
        for _ in range(250):
            r = self.raw(motor_id, mode=1, kd=10)
            if r and abs(r.vel) < 15:
                break
        r = None
        for _ in range(20):
            r = self.raw(motor_id) or r
        return r

    def go_centre(self, motor_id, speed=50_000):
        j = self.joints[motor_id]
        r = None
        for _ in range(10):
            r = self.raw(motor_id, mode=1) or r
        j.sync(r.pos)
        target = (j.low + j.high) / 2
        x, t_last = float(r.pos), time.monotonic()
        while abs(target - x) > 1:
            now = time.monotonic()
            step = min(speed * (now - t_last), abs(target - x))
            x, t_last = x + math.copysign(step, target - x), now
            self.raw(motor_id, mode=1, pos=round(x), kp=300, kd=KD[motor_id])
        for _ in range(150):
            self.raw(motor_id, mode=1, pos=round(target), kp=300, kd=KD[motor_id])
        return self.rest(motor_id)


rig = Rig()
bus = rig.bus

# ---------------------------------------------------------------- 1. bus basics
found = bus.scan()
check("scan finds the motors; idle reply mode 0", sorted(found) == IDS and all(s.mode == 0 for s in found.values()),
      f"ids {sorted(found)}, modes {[s.mode for s in found.values()]}")
if sorted(found) != IDS:
    sys.exit("expected ids 4, 5, 6 on a bus without a controller")
if any(s.fault for s in found.values()):
    print("a motor reports a fault; restarting all three first")
    bus.restart(IDS)
    found = bus.scan()
check("status word 0 in normal operation", all(s.error == 0 for s in found.values()), str([hex(s.error) for s in found.values()]))
check("bus voltage 0.5 V per count: 24.0-24.5 V on the 24 V supply", all(23.5 <= s.voltage <= 25.0 for s in found.values()),
      str([s.voltage for s in found.values()]))
check("coil temperature >= shell temperature", all(s.temp_coil >= s.temp_shell for s in found.values()),
      str([(s.temp_coil, s.temp_shell) for s in found.values()]))
absent = [i for i in (0, 1, 2, 3, 7, 8) if bus.transfer(MotorCmd(id=i)) is not None]
check("motors transmit only when addressed (absent ids stay silent)", not absent, "no reply for ids 0-3, 7, 8" if not absent else f"replies from {absent}")
t0, n = time.monotonic(), 0
while time.monotonic() - t0 < 1.0:
    for i in IDS:
        bus.transfer(MotorCmd(id=i))
    n += 1
check("polling rate from Python, three motors (guide: about 340 per motor)", 250 < n < 450, f"{n} per second per motor")
ok = all(bus.transfer(MotorCmd(id=i)) is not None for _ in range(300) for i in IDS)
check("a motor answers every command", ok, "900 of 900 idle polls answered" if ok else "missed replies")

# ---------------------------------------------------------------- 2. enabling
try:
    st = bus.enable(IDS)
    check("enable sequence (idle, FLAG, MODE 1) reaches run mode; end stops fit after re-referencing",
          all(s.mode == 1 for s in st.values()), "modes " + str([s.mode for s in st.values()])
          + "; end stops " + ", ".join(f"id{i} {rig.joints[i].low}..{rig.joints[i].high}" for i in IDS))
except Exception as e:  # noqa: BLE001
    check("enable sequence reaches run mode", False, str(e))
    sys.exit(1)
bus.disable(IDS)
modes = []
for _ in range(5):
    r = bus.transfer(MotorCmd(id=T, mode=MODE_RUN, flag=False, byte3=0))
    modes.append(r.mode if r else None)
check("run mode is entered without FLAG, sequence byte or preamble", modes[-1] == 1,
      f"MODE 1 with FLAG clear and byte 3 = 0 straight from idle: reply modes {modes}")
rig.rest(T)

# ---------------------------------------------------------------- 3. command fields in run mode (motor 4)
def trial(mode, seconds=0.4, vmax=1200, **fields):
    start = rig.go_centre(T)
    f = dict(fields)
    if "dpos" in f:
        f["pos"] = start.pos + f.pop("dpos")
    t0, vels, taus, last, modes = time.monotonic(), [], [], start, set()
    while time.monotonic() - t0 < seconds:
        r = rig.raw(T, mode=mode, **f)
        if r:
            last = r; vels.append(r.vel); taus.append(r.tau); modes.add(r.mode)
            if abs(r.vel) > vmax or abs(r.pos - start.pos) > 40_000:
                break
    out = dict(moved=last.pos - start.pos, peak=max(vels, key=abs) if vels else 0, end=statistics.mean(vels[-20:]) if vels else 0,
               tau=statistics.mean(taus) if taus else 0, modes=modes, status=last.error)
    rig.rest(T)
    return out

a, b = trial(1, tau=15), trial(1, tau=-15)
check("T is a torque feed-forward: +/-15 counts accelerates the joint both ways", a["peak"] > 800 and b["peak"] < -800, f"peak velocity {a['peak']} and {b['peak']} counts")
a = trial(1, dpos=4000, kp=100)
check("POS/KP: a 4,000-count step with KP 100 moves the joint toward the target", 800 < a["moved"] < 4500, f"moved {a['moved']} counts (friction decides where it settles)")
a, b = trial(1, vel=300, kd=20), trial(1, vel=-300, kd=20)
check("W/KD: velocity command +/-300 with KD 20 is followed", 120 < a["end"] < 380 and -380 < b["end"] < -120, f"steady velocity {a['end']:.0f} and {b['end']:.0f} counts")
a, b = trial(1, tau=15, kd=20), trial(1, kd=60)
check("KD damps: torque 15 with KD 20 stays slow; KD alone does not move the joint", abs(a["peak"]) < 400 and abs(b["moved"]) < 600,
      f"peak velocity {a['peak']} with torque+KD; moved {b['moved']} with KD only")

# ---------------------------------------------------------------- 4. mode table (motor 4)
baseline = {}
for mode, accepted in ((2, False), (6, False), (3, True), (4, True), (5, True)):
    a = trial(mode, seconds=0.25)
    baseline[mode] = a["peak"]
    got = mode in a["modes"]
    check(f"mode {mode} with zero payload: {'accepted' if accepted else 'ignored'}, no motion", got == accepted and abs(a["peak"]) < 300,
          f"reply modes {sorted(a['modes'])}, peak velocity {a['peak']}")
for mode in (3, 4, 5):
    a = trial(mode, tau=15)
    check(f"mode {mode} ignores command fields (torque +15)", abs(a["peak"] - baseline[mode]) < 200,
          f"peak velocity {a['peak']}, against {baseline[mode]} with a zero payload (mode 1 gave over 800)")


def coast(mode, sign):
    start = rig.go_centre(T)
    r, t0 = start, time.monotonic()
    while time.monotonic() - t0 < 0.05:
        r = rig.raw(T, mode=1, tau=15 * sign) or r
        if abs(r.vel) >= 800:
            break
    t1, stop = time.monotonic(), None
    while time.monotonic() - t1 < 0.3:
        r2 = rig.raw(T, mode=mode, flag=(mode != 0))
        if r2 and stop is None and abs(r2.vel) < 50:
            stop = time.monotonic() - t1
    rig.rest(T)
    return (stop or 0.3) * 1000


cd = {m: [coast(m, +1), coast(m, -1)] for m in (0, 1, 3, 4, 5)}
check("idle (mode 0) brakes the joint: stops from about 800 velocity counts in about 10 ms", max(cd[0]) < 20, f"{cd[0][0]:.0f} / {cd[0][1]:.0f} ms")
check("modes 3 and 5 brake like idle", max(cd[3] + cd[5]) < 20, f"mode 3: {cd[3][0]:.0f} / {cd[3][1]:.0f} ms, mode 5: {cd[5][0]:.0f} / {cd[5][1]:.0f} ms")
check("mode 1 (zero command) and mode 4 leave the joint free (guide: 25-64 ms)", min(cd[1] + cd[4]) > 18,
      f"mode 1: {cd[1][0]:.0f} / {cd[1][1]:.0f} ms, mode 4: {cd[4][0]:.0f} / {cd[4][1]:.0f} ms")

# damping torque relation: steady speed under a constant torque T with damping KD
cs = []
for torque, kd in ((20, 20), (40, 80)):
    v = []
    for sign in (1, -1):
        start = rig.go_centre(T)
        t0, vels = time.monotonic(), []
        while time.monotonic() - t0 < 0.35:
            r = rig.raw(T, mode=1, tau=torque * sign, kd=kd)
            if r:
                vels.append(r.vel)
                if abs(r.pos - start.pos) > 45_000:
                    break
        rig.rest(T)
        v.append(statistics.mean(vels[len(vels) // 2:]))
    cs.append((torque - 4) / (kd * (v[0] - v[1]) / 2))       # 4 counts of friction on this joint
check("damping torque is 0.005 x KD x velocity reading", all(0.004 < c < 0.006 for c in cs), f"{cs[0]:.5f} at T 20 / KD 20, {cs[1]:.5f} at T 40 / KD 80")

# ---------------------------------------------------------------- 5. hold, and what happens when polling stops
st = bus.enable(IDS)
for _ in range(150):                                   # let the joints settle in zero torque first
    for i in IDS:
        st[i] = bus.command(i) or st[i]
target = {i: st[i].pos for i in IDS}
worst = {i: 0 for i in IDS}
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:
    for i in IDS:
        s = bus.command(i, pos=target[i], kp=300, kd=KD[i])
        if s:
            worst[i] = max(worst[i], abs(s.pos - target[i]))
check("position hold with KP 300 stays on target", max(worst.values()) < 100, f"largest error {worst} counts in 2 s")
time.sleep(0.3)                                        # no polling at all
short = {i: bus.command(i, pos=target[i], kp=300, kd=KD[i]) for i in IDS}
for _ in range(30):
    for i in IDS:
        bus.command(i, pos=target[i], kp=300, kd=KD[i])
time.sleep(1.2)
long_ = {i: bus.command(i, pos=target[i], kp=300, kd=KD[i]) for i in IDS}
check("without polling a motor stays in run mode for 0.3 s and has left it after 1.2 s",
      all(short[i] and short[i].mode == 1 for i in IDS) and all(long_[i] and long_[i].mode == 0 and long_[i].error == 0 for i in IDS),
      f"reply mode after 0.3 s {[short[i].mode for i in IDS]}, after 1.2 s {[long_[i].mode for i in IDS]}, "
      f"status {[hex(long_[i].error) for i in IDS]}, moved {[long_[i].pos - target[i] for i in IDS]} counts")
bus.disable(IDS)

# ---------------------------------------------------------------- 6. restart (mode 7)
for i in IDS:
    before = None
    for _ in range(5):
        before = bus.transfer(MotorCmd(id=i)) or before
    t0 = time.monotonic()
    bus.transfer(MotorCmd(id=i, mode=MODE_RESTART))
    last_reply, back, after = 0.0, None, None
    while time.monotonic() - t0 < 1.0:
        r = bus.transfer(MotorCmd(id=i))
        t = time.monotonic() - t0
        if r and back is None and t < 0.1:
            last_reply = t
        elif r and t >= 0.1 and back is None:
            back, after = t, r
    j = rig.joints[i]
    shift = (after.pos - before.pos) if after else 0
    k = round(shift / j.turn_counts)
    okk = after is not None and after.mode == 0 and after.error == 0 and 0.2 < back < 0.45 and abs(shift - k * j.turn_counts) < 3000
    check(f"mode 7 restarts id {i}: silent until about 300 ms, back in mode 0 with status 0", okk,
          f"answered until {last_reply * 1000:.0f} ms, back at {back * 1000:.0f} ms; position shift {shift:+d} = {k:+d} output turn(s) "
          f"of {j.turn_counts:,}; coil/shell {after.temp_coil}/{after.temp_shell}" if after else "no reply after restart")
    if after:
        j.sync(after.pos)
bus.close(brake=False)

# ---------------------------------------------------------------- 7. motion runs through sweep.py
def sweep(name, *extra):
    log = os.path.join(LOGDIR, time.strftime("%H%M%S") + f"_verify_{name}.csv")
    p = subprocess.run([PY, os.path.join(ROOT, "scripts", "sweep.py"), "--kp", "300", "--kd", "40,80,80", "--ff-coulomb", "3.7,18.7,11.6",
                        "--ff-viscous", "0,1.5,6.0", "--log", log, *extra], capture_output=True, text=True, timeout=300)
    rows = list(csv.DictReader(open(log))) if os.path.exists(log) else []
    return p, rows


def per_id(rows, fn):
    return {i: fn([r for r in rows if int(r["id"]) == i]) for i in IDS}


def scan_status():
    with MotorBus() as b:
        return {i: s.error for i, s in b.scan().items()}


if not args.skip_motion_runs:
    p, rows = sweep("nudge_weak", "--nudge", "5000", "--kp", "20", "--kd", "1")
    run = [r for r in rows if r["phase"] == "run"]
    lag = per_id(run, lambda m: max(abs(float(r["target"]) - int(r["pos"])) for r in m))
    check("KP 20, KD 1 is too weak to track on ids 5 and 6 (+/-5,000-count nudge)", lag[5] > 3000 and lag[6] > 3000,
          f"peak lag {', '.join(f'id{i} {lag[i]:.0f}' for i in IDS)} counts (the joint stays behind or drifts, depending on the pose)")

    p, rows = sweep("sine_8s", "--sine", "24", "--period", "8")
    run = [r for r in rows if r["phase"] == "run"]
    def ripple(m):
        import numpy as np
        t = np.array([float(r["t"]) for r in m]); tgt = np.array([float(r["target"]) for r in m]); vel = np.array([int(r["vel"]) * 128 for r in m])
        tv = np.gradient(tgt, t); mv = np.abs(tv) > 5000
        return 100 * np.std((vel - tv)[mv]) / np.abs(tv).max()
    rp = per_id(run, ripple)
    lag = per_id(run, lambda m: max(abs(float(r["target"]) - int(r["pos"])) for r in m))
    check("KP 300, KD 40/80/80 tracks an 8 s full-range sine smoothly (guide: ripple 3-4 %)", "complete" in p.stdout and max(rp.values()) < 6,
          f"ripple {', '.join(f'id{i} {rp[i]:.1f} %' for i in IDS)}; peak lag {', '.join(f'id{i} {lag[i]:.0f}' for i in IDS)}")

    p, rows = sweep("sine_0.8s", "--sine", "6", "--period", "0.8", "--max-lag", "25000")
    run = [r for r in rows if r["phase"] == "run"]
    peak = per_id(run, lambda m: max(abs(int(r["vel"])) for r in m) * 128)
    lag = per_id(run, lambda m: max(abs(float(r["target"]) - int(r["pos"])) for r in m))
    check("0.8 s full-range sine still tracks; speed tops out near 1,275,000 counts/s on ids 5 and 6", "complete" in p.stdout and all(1.15e6 < peak[i] < 1.4e6 for i in (5, 6)),
          f"measured peak {', '.join(f'id{i} {peak[i]:,}' for i in IDS)}; peak lag {', '.join(f'id{i} {lag[i]:.0f}' for i in IDS)}; "
          + ("completed" if "complete" in p.stdout else p.stdout.strip().splitlines()[-2]))

    p, rows = sweep("abort_idle_brake", "--sine", "6", "--period", "0.8", "--abort-at", "2.4")
    brake = [r for r in rows if r["phase"] == "idle"]
    v0 = per_id(brake, lambda m: abs(int(m[0]["vel"])) * 128 if m else 0)
    pk = per_id(brake, lambda m: max(abs(int(r["tau"])) for r in m) if m else 0)
    vmin = per_id(brake, lambda m: min(int(r["volt_raw"]) for r in m) * 0.5 if m else 0)
    st = scan_status()
    seen = sorted({r["status"] for r in brake})
    check("stopping from top speed by going to idle: no fault, supply voltage steady", all(v == 0 for v in st.values()) and seen == ["0"]
          and max(v0.values()) > 9e5 and min(vmin.values()) >= 23.0,
          f"speed at abort {', '.join(f'id{i} {v0[i]:,}' for i in IDS)}; peak torque reading {pk}; lowest voltage {vmin}; status during stop {seen}")

if args.fault:
    p, rows = sweep("abort_hard_brake", "--sine", "6", "--period", "0.8", "--abort-at", "2.4", "--hard-brake")
    brake = [r for r in rows if r["phase"] == "brake"]
    pk = per_id(brake, lambda m: max(abs(int(r["tau"])) for r in m) if m else 0)
    vmin = per_id(brake, lambda m: min(int(r["volt_raw"]) for r in m) * 0.5 if m else 0)
    st = scan_status()
    check("hard brake (KD 80) from top speed latches status 0x8 with a supply sag", all(v == 8 for v in st.values()),
          f"peak brake torque {pk}; lowest voltage {vmin}; status {[hex(v) for v in st.values()]}")
    with MotorBus() as b:
        for _ in range(300):
            for i in IDS:
                b.transfer(MotorCmd(id=i))
        refused = {}
        for i in IDS:
            r = None
            for n in range(40):
                r = b.transfer(MotorCmd(id=i, mode=MODE_RUN, flag=True, byte3=0x80)) or r
            refused[i] = r.mode if r else None
        still = {i: s.error for i, s in b.scan().items()}
        check("a latched fault survives idle polling and blocks MODE 1", all(v == 8 for v in still.values()) and all(m == 0 for m in refused.values()),
              f"status {[hex(v) for v in still.values()]}, reply mode to MODE 1 {list(refused.values())}")
        after = b.restart(IDS)
        check("mode 7 clears the fault", all(s.error == 0 for s in after.values()), f"status {[hex(s.error) for s in after.values()]}")
        en = b.enable(IDS)
        check("motors enter run mode again after the restart", all(s.mode == 1 for s in en.values()), str([s.mode for s in en.values()]))

good = sum(1 for _, ok, _ in results if ok)
bad = [n for n, ok, _ in results if ok is False]
print(f"\n{good} passed, {len(bad)} failed, {sum(1 for _, ok, _ in results if ok is None)} informational")
for n in bad:
    print("  FAILED:", n)
sys.exit(1 if bad else 0)
