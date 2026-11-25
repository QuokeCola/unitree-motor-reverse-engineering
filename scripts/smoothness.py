#!/usr/bin/env python3
"""Smoothness figures for a sweep.py log: lag, velocity ripple, torque chatter per joint."""
import sys

import numpy as np

for path in sys.argv[1:]:
    a = np.genfromtxt(path, delimiter=",", skip_header=1)
    print(path.split("/")[-1])
    for i in np.unique(a[:, 1]).astype(int):
        m = a[a[:, 1] == i]
        t, tgt, pos, vel, tau = m[:, 0], m[:, 2], m[:, 3], m[:, 4] * 128, m[:, 5]
        tv = np.gradient(tgt, t)
        moving = np.abs(tv) > 5000
        ve = (vel - tv)[moving]
        flips = np.sum(np.diff(np.sign(tau[moving])) != 0) / max(moving.sum() * np.median(np.diff(t)), 1e-9)
        fit = np.linalg.lstsq(np.c_[np.sign(tv), tv / 1e5, np.ones(len(t))][moving], (tgt - pos)[moving], rcond=None)[0]
        print(f"  id {i}: lag from friction {fit[0]:5.0f} + {fit[1]:5.0f} per 1e5 counts/s |", end="")
        print(f" lag rms {np.std((tgt - pos)[moving]):6.0f}  peak {np.abs(tgt - pos).max():6.0f}   "
              f"velocity ripple rms {ve.std():6.0f} counts/s ({100 * ve.std() / np.abs(tv).max():4.1f}% of peak)   "
              f"torque sign flips {flips:5.1f}/s   peak torque {np.abs(tau).max():.0f}")
