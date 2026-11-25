#!/usr/bin/env python3
"""Spin one joint up briefly in run mode, then switch to another mode with an all-zero command
and record how it slows down. Shows whether a mode leaves the joint free, damps it or holds it."""
import statistics
import sys
import time

sys.argv = [sys.argv[0], "--id", sys.argv[1] if len(sys.argv) > 1 else "4", "--modes", "1"]
import mode_characterize as mc   # reuses its port, limits check, go_to() and xfer(); runs its mode-1 table first

MODES = [0, 1, 3, 4, 5]
print("\ncoast-down from about 800 velocity counts, all-zero command in the test mode")
print(f"{'mode':>4} {'dir':>4} {'reply mode':>10} {'start vel':>9} {'vel after 10/20/40/80 ms':>26} {'time to <50':>12} {'travel':>8} {'mean tau':>9}")
ser = mc.serial.Serial(mc.PORT, 4_000_000, timeout=0.0005)
mc.set_baud(ser.fd, 6_000_000)
ser.set_low_latency_mode(True)
time.sleep(0.1)
mc.ser = ser
for mode in MODES:
    for sign in (1, -1):
        start = mc.go_to(mc.centre)
        r = start
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.05:                      # spin up
            r = mc.xfer(mode=1, tau=15 * sign) or r
            if abs(r["vel"]) >= 800:
                break
        v0, p0, t1 = r["vel"], r["pos"], time.monotonic()
        samples, modes, taus = [], set(), []
        while time.monotonic() - t1 < 0.3:
            r2 = mc.xfer(mode=mode, flag=(mode != 0)) 
            if r2:
                samples.append((time.monotonic() - t1, r2["vel"], r2["pos"])); modes.add(r2["mode"]); taus.append(r2["tau"])
                if abs(r2["pos"] - mc.centre) > 60_000:
                    break
        def at(ms):
            c = [v for t, v, _ in samples if t >= ms / 1000]
            return c[0] if c else 0
        slow = next((t for t, v, _ in samples if abs(v) < 50), None)
        travel = samples[-1][2] - p0 if samples else 0
        print(f"{mode:>4} {'+' if sign > 0 else '-':>4} {str(sorted(modes)):>10} {v0:>9d} "
              f"{at(10):>6d}/{at(20):>5d}/{at(40):>5d}/{at(80):>5d} {('%.0f ms' % (slow * 1000)) if slow else 'never':>12} {travel:>8d} "
              f"{statistics.mean(taus) if taus else 0:>9.1f}", flush=True)
        mc.settle()
mc.go_to(mc.centre)
mc.poll(30)
ser.close()
