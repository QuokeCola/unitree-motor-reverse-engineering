"""!
@file joint.py
@brief Joint-level view of a motor: gear ratio, end stops, software zero.
"""
from __future__ import annotations

import dataclasses
import json
import math

## Position counts in one turn of the motor rotor.
COUNTS_PER_ROTOR_TURN = 32768
## Position counts per second represented by one velocity count.
POS_COUNTS_PER_VEL_COUNT = 128.0
## Gear ratios seen on one G1 wrist (ids 4, 5, 6); id 5 is assumed.
WRIST_GEAR_RATIOS = {4: 16, 5: 25, 6: 25}


@dataclasses.dataclass
class Joint:
    """!
    @brief Gear ratio, end stops and zero of one joint, in raw counts.

    After a power cycle or restart a motor's position reading can come back
    shifted by a whole number of output-shaft turns. sync() moves the stored
    zero and end stops by the same amount.
    """

    ## Rotor turns per output turn.
    gear_ratio: float
    ## Lower end stop, or None.
    low: int | None = None
    ## Upper end stop, or None.
    high: int | None = None
    ## Count that is angle 0; defaults to the middle of the travel.
    zero: int | None = None
    ## Counts kept clear of each end stop by clamp().
    margin: int = 0
    ## Slack allowed by sync().
    tolerance: int = 3000

    def __post_init__(self):
        """!
        @brief Checks the end stops and fills in the default zero.
        @exception ValueError Only one end stop given, or low not below high.
        """
        if (self.low is None) != (self.high is None):
            raise ValueError("give both end stops or neither")
        if self.low is not None and self.low >= self.high:
            raise ValueError("low must be below high")
        if self.zero is None:
            if self.low is None:
                self.zero = 0
            else:
                self.zero = round((self.low + self.high) / 2)

    @property
    def turn_counts(self) -> int:
        """!
        @brief Size of one output turn.
        @return Position counts in one turn of the output shaft.
        """
        return round(self.gear_ratio * COUNTS_PER_ROTOR_TURN)

    @property
    def travel(self) -> float | None:
        """!
        @brief Angle between the end stops.
        @return Travel in rad, or None without end stops.
        """
        if self.low is None:
            return None
        return (self.high - self.low) / self.turn_counts * 2 * math.pi

    def sync(self, pos: int) -> int:
        """!
        @brief Shifts zero and end stops by whole output turns to match a
               position reading.

        Call after every power-on or restart.

        @param pos Current position reading.
        @return Number of output turns shifted.
        @exception ValueError The reading still misses the end stops.
        """
        if self.low is None:
            reference = self.zero
        else:
            reference = (self.low + self.high) / 2
        turns = round((pos - reference) / self.turn_counts)
        shift = turns * self.turn_counts
        self.zero += shift
        if self.low is not None:
            self.low += shift
            self.high += shift
            if not (self.low - self.tolerance <= pos
                    <= self.high + self.tolerance):
                raise ValueError(f"position {pos} does not fit the end stops "
                                 f"{self.low} .. {self.high}")
        return turns

    def angle(self, pos: float) -> float:
        """!
        @brief Converts a position to an output-shaft angle.
        @param pos Position in counts.
        @return Angle in rad from the zero.
        """
        return (pos - self.zero) / self.turn_counts * 2 * math.pi

    def counts(self, angle: float) -> int:
        """!
        @brief Converts an output-shaft angle to a position.
        @param angle Angle in rad from the zero.
        @return Position in counts.
        """
        turns = angle / (2 * math.pi)
        return round(self.zero + self.turn_counts * turns)

    def speed(self, vel: float) -> float:
        """!
        @brief Converts a velocity reading to output-shaft speed.
        @param vel Velocity in counts.
        @return Speed in rad/s.
        """
        return (vel * POS_COUNTS_PER_VEL_COUNT / self.turn_counts
                * 2 * math.pi)

    def clamp(self, pos: float) -> int:
        """!
        @brief Limits a position to the end stops less the margin.
        @param pos Position in counts.
        @return The limited position.
        """
        if self.low is None:
            return round(pos)
        return round(min(max(pos, self.low + self.margin),
                         self.high - self.margin))


def load_joints(path: str, gear_ratios: dict[int, float] | None = None,
                margin: int = 0) -> dict[int, Joint]:
    """!
    @brief Reads joints from an end-stop file written by `g1motor limits`.

    @param path JSON file of the form {"4": [low, high], ...}.
    @param gear_ratios Gear ratio per id; defaults to WRIST_GEAR_RATIOS.
    @param margin Margin given to every joint.
    @return Joints by motor id.
    """
    if gear_ratios is None:
        gear_ratios = WRIST_GEAR_RATIOS
    with open(path) as limits_file:
        data = json.load(limits_file)
    return {int(motor_id): Joint(gear_ratios[int(motor_id)], low=low,
                                 high=high, margin=margin)
            for motor_id, (low, high) in data.items()}
