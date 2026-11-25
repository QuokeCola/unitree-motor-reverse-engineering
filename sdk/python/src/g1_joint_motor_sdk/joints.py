"""Joint-level view of a motor: gear ratio, end stops, a software zero, and the whole-turn
ambiguity of the position reading.

A motor's position reading is multi-turn, but after a power cycle or a restart it can come back
shifted by a whole number of OUTPUT-shaft turns (gear_ratio rotor turns). The motor therefore
knows where the joint is within one output turn, and a zero or an end stop stored in raw counts
stays valid once it is shifted by the same whole number of turns. Joint.sync() does that.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

COUNTS_PER_ROTOR_TURN = 32768
# Gear ratios seen on one G1 wrist assembly (motor ids 4, 5, 6). 16 and 25 come from the size of
# the position jump after a restart; id 5 never jumped and is assumed equal to id 6 (same travel).
WRIST_GEAR_RATIOS = {4: 16, 5: 25, 6: 25}


@dataclass
class Joint:
    gear_ratio: float
    low: int | None = None      # end stop, raw counts
    high: int | None = None     # end stop, raw counts
    zero: int | None = None     # raw count that is angle 0; default: middle of travel, else 0
    margin: int = 0             # counts kept clear of each end stop by clamp()
    tolerance: int = 3000       # how far outside the stored end stops a reading may be in sync()

    def __post_init__(self):
        if (self.low is None) != (self.high is None):
            raise ValueError("give both end stops or neither")
        if self.low is not None and self.low >= self.high:
            raise ValueError("low must be below high")
        if self.zero is None:
            self.zero = round((self.low + self.high) / 2) if self.low is not None else 0

    @property
    def turn_counts(self) -> int:
        """Position counts in one turn of the output shaft."""
        return round(self.gear_ratio * COUNTS_PER_ROTOR_TURN)

    def sync(self, pos: int) -> int:
        """Shift zero and end stops by whole output turns so they match this position reading.

        Call after every power-on or restart. Returns the number of turns shifted. Raises
        ValueError if the reading still falls outside the end stops: they must be re-measured.
        """
        reference = (self.low + self.high) / 2 if self.low is not None else self.zero
        k = round((pos - reference) / self.turn_counts)
        shift = k * self.turn_counts
        self.zero += shift
        if self.low is not None:
            self.low += shift
            self.high += shift
            if not self.low - self.tolerance <= pos <= self.high + self.tolerance:
                raise ValueError(f"position {pos} does not fit the end stops {self.low} .. {self.high}")
        return k

    def angle(self, pos: float) -> float:
        """Output-shaft angle in rad from the zero."""
        return (pos - self.zero) / self.turn_counts * 2 * math.pi

    def counts(self, angle: float) -> int:
        """Raw position for an output-shaft angle in rad."""
        return round(self.zero + angle / (2 * math.pi) * self.turn_counts)

    def speed(self, vel: float) -> float:
        """Output-shaft speed in rad/s for a raw velocity reading."""
        return vel * 128 / self.turn_counts * 2 * math.pi

    def clamp(self, pos: float) -> int:
        if self.low is None:
            return round(pos)
        return round(min(max(pos, self.low + self.margin), self.high - self.margin))

    @property
    def travel(self) -> float | None:
        """Angle between the end stops in rad."""
        return None if self.low is None else (self.high - self.low) / self.turn_counts * 2 * math.pi


def load_joints(path: str, gear_ratios: dict[int, float] = WRIST_GEAR_RATIOS, margin: int = 0) -> dict[int, Joint]:
    """Joints from an end-stop file written by `g1motor limits` ({"4": [low, high], ...})."""
    data = json.load(open(path))
    return {int(i): Joint(gear_ratios[int(i)], low=lo, high=hi, margin=margin) for i, (lo, hi) in data.items()}
