#!/usr/bin/env python3
"""!
@file joint_test.py
@brief Tests of Joint: whole-turn re-referencing, angles, clamping. No
       hardware.
"""
import math
import sys

import pytest

from g1motor import Joint


def test_turn_counts():
    """!
    @brief Counts per output turn for 16:1 and 25:1.
    """
    assert Joint(16).turn_counts() == 524288
    assert Joint(25).turn_counts() == 819200


def test_sync_follows_whole_turn_shift():
    """!
    @brief sync() follows a reading shifted by one output turn.
    """
    # End stops measured before a restart; the reading then came back one
    # output turn higher (bench id 6).
    joint = Joint(25, low=-188529, high=236437)
    zero = joint.zero
    assert joint.sync(830810) == 1
    assert joint.low == -188529 + 819200
    assert joint.high == 236437 + 819200
    assert joint.zero == zero + 819200
    assert joint.sync(830810) == 0


def test_sync_rejects_reading_outside_end_stops():
    """!
    @brief sync() rejects a reading beyond the end stops.
    """
    joint = Joint(16, low=-94259, high=236635)
    with pytest.raises(ValueError):
        joint.sync(236635 + 100_000)


def test_angle_round_trip_and_travel():
    """!
    @brief Angle and counts convert back and forth; travel in degrees.
    """
    joint = Joint(16, low=-94259, high=236635)
    assert joint.angle(joint.zero) == 0
    assert joint.counts(joint.angle(123456)) == 123456
    assert math.degrees(joint.travel()) == pytest.approx(227.2, abs=0.1)
    assert joint.angle(joint.zero + 524288) == pytest.approx(2 * math.pi)


def test_clamp_uses_margin():
    """!
    @brief clamp() keeps the margin from each end stop.
    """
    joint = Joint(25, low=0, high=400000, margin=40000)
    assert joint.clamp(-5) == 40000
    assert joint.clamp(999999) == 360000
    assert joint.clamp(123456.4) == 123456


def test_speed():
    """!
    @brief Output speed for the 24 V speed ceiling on bench id 4.
    """
    # The 24 V speed ceiling on bench id 4, in rad/s at the output.
    assert Joint(16).speed(9960) == pytest.approx(15.28, abs=0.02)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
