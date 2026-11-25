import math

import pytest

from g1_joint_motor_sdk import Joint


def test_turn_counts():
    assert Joint(16).turn_counts == 524288 and Joint(25).turn_counts == 819200


def test_sync_follows_whole_turn_shift():
    # end stops measured before a restart; the reading then came back one output turn higher (bench id 6)
    j = Joint(25, low=-188529, high=236437)
    zero = j.zero
    assert j.sync(830810) == 1
    assert (j.low, j.high, j.zero) == (-188529 + 819200, 236437 + 819200, zero + 819200)
    assert j.sync(830810) == 0                       # already in step


def test_sync_rejects_reading_outside_end_stops():
    j = Joint(16, low=-94259, high=236635)
    with pytest.raises(ValueError):
        j.sync(236635 + 100_000)                     # nearer this turn than the next, but beyond the stop


def test_angle_round_trip_and_travel():
    j = Joint(16, low=-94259, high=236635)
    assert j.angle(j.zero) == 0
    assert j.counts(j.angle(123456)) == 123456
    assert math.degrees(j.travel) == pytest.approx(227.2, abs=0.1)
    assert j.angle(j.zero + 524288) == pytest.approx(2 * math.pi)


def test_clamp_uses_margin():
    j = Joint(25, low=0, high=400000, margin=40000)
    assert j.clamp(-5) == 40000 and j.clamp(999999) == 360000 and j.clamp(123456.4) == 123456


def test_speed():
    assert Joint(16).speed(9960) == pytest.approx(15.28, abs=0.02)   # the 24 V speed ceiling on id 4, rad/s
