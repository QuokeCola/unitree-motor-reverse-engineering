#!/usr/bin/env python3
"""Build a position-dependent torque feed-forward table from a slow sweep.py sine log.

For each joint and each direction of travel, the torque the motor reported while tracking
is averaged in position bins. sweep.py --ff-table then feeds that torque forward.

  build_ff_table.py LOG.csv OUT.json [--bin 1024]
"""
import argparse
import json

import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("log")
ap.add_argument("out")
ap.add_argument("--bin", type=int, default=1024, help="bin width in position counts")
ap.add_argument("--min-speed", type=float, default=8000, help="ignore samples slower than this (counts/s)")
args = ap.parse_args()

a = np.genfromtxt(args.log, delimiter=",", skip_header=1)
out = {}
for i in np.unique(a[:, 1]).astype(int):
    m = a[a[:, 1] == i]
    t, tgt, pos, tau = m[:, 0], m[:, 2], m[:, 3], m[:, 5]
    tv = np.gradient(tgt, t)
    lo, hi = tgt.min(), tgt.max()
    edges = np.arange(lo, hi + args.bin, args.bin)
    centres = 0.5 * (edges[:-1] + edges[1:])
    entry = {"start": float(centres[0]), "step": float(args.bin)}
    for key, sel in (("up", tv > args.min_speed), ("down", tv < -args.min_speed)):
        which = np.digitize(pos[sel], edges) - 1
        ok = (which >= 0) & (which < len(centres))
        total = np.bincount(which[ok], weights=tau[sel][ok], minlength=len(centres))
        count = np.bincount(which[ok], minlength=len(centres))
        have = count > 0
        mean = np.interp(centres, centres[have], total[have] / count[have])   # fill empty bins
        mean = np.convolve(np.pad(mean, 2, mode="edge"), np.ones(5) / 5, mode="valid")   # 5-bin smoothing
        entry[key] = [round(float(x), 2) for x in mean]
        print(f"id {i} {key:4s}: {have.sum()}/{len(centres)} bins filled, {count[have].mean():.1f} samples per bin, "
              f"torque mean {mean.mean():6.1f}, min {mean.min():6.1f}, max {mean.max():6.1f}")
    out[str(i)] = entry
json.dump(out, open(args.out, "w"))
print("wrote", args.out)
