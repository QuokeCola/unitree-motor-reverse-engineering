#!/usr/bin/env python3
"""!
@file protocol_test.py
@brief Frame tests against frames captured from a G1 arm bus. No hardware.
"""
import sys

import pytest

import g1motor

## Run mode, zero torque, id 0 (robot).
CMD_ZERO_TORQUE = bytes.fromhex("feee90800000000000000000000000009b1e4106")
## Run mode, kd 25, id 0 (robot damping).
CMD_DAMPING = bytes.fromhex("feee9080000000000000000000001900541ed614")
## Idle poll, id 0.
CMD_IDLE = bytes.fromhex("feee000000000000000000000000000080462079")
## Hold 25,032 with kp 450, kd 10, id 0 (robot standing).
CMD_HOLD = bytes.fromhex("feee908000000000c8610000c2010a0071dc7c64")
## Restart, id 0 (robot, after calibration).
CMD_RESTART = bytes.fromhex("feee7000000000000000000000000000378838d7")
## Reply of robot id 2 in run mode.
REPLY_ROBOT = bytes.fromhex(
    "fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c")
## Reply of bench id 4 in idle on 24 V.
REPLY_BENCH = bytes.fromhex(
    "fcee041e1c30fbfff9ff1b08010000000000c00b000057221ecf")
## Reply of bench id 4 with status 0x8.
REPLY_FAULT = bytes.fromhex(
    "fcee042626300000fdffc15d010008000000a80b0000a190371a")


def test_checksum():
    """!
    @brief Checksums of a captured command and reply.
    """
    assert (g1motor.crc32_core(CMD_ZERO_TORQUE[:16]).to_bytes(4, "little")
            == CMD_ZERO_TORQUE[16:])
    assert (g1motor.crc32_core(REPLY_ROBOT[2:22]).to_bytes(4, "little")
            == REPLY_ROBOT[22:])


def test_encode_matches_capture():
    """!
    @brief Encoded commands equal the captured frames.
    """
    run = dict(id=0, mode=g1motor.MODE_RUN, flag=True, byte3=0x80)
    assert g1motor.MotorCmd(id=0).encode() == CMD_IDLE
    assert g1motor.MotorCmd(**run).encode() == CMD_ZERO_TORQUE
    assert g1motor.MotorCmd(**run, kd=25).encode() == CMD_DAMPING
    assert g1motor.MotorCmd(**run, pos=25032, kp=450, kd=10).encode() == CMD_HOLD
    assert g1motor.MotorCmd(id=0, mode=g1motor.MODE_RESTART).encode() == CMD_RESTART


def test_command_round_trip():
    """!
    @brief A command survives encode and decode.
    """
    cmd = g1motor.MotorCmd(id=6, mode=g1motor.MODE_RUN, flag=True, byte3=0x90,
                       tau=-12, vel=345, pos=-123456, kp=300, kd=80)
    assert g1motor.MotorCmd.decode(cmd.encode()) == cmd


def test_decode_reply():
    """!
    @brief Fields of captured robot and bench replies.
    """
    state = g1motor.MotorState.decode(REPLY_ROBOT)
    assert (state.id, state.mode, state.pos) == (2, 1, -180794)
    assert (state.vel, state.tau) == (-8, -1)
    assert (state.temp_coil, state.temp_shell, state.voltage()) == (36, 35, 48.5)
    assert not state.fault()
    bench = g1motor.MotorState.decode(REPLY_BENCH)
    assert (bench.id, bench.mode, bench.pos) == (4, g1motor.MODE_IDLE, 67611)
    assert bench.voltage() == 24.0


def test_fault_word():
    """!
    @brief A captured reply with status 0x8 reports a fault.
    """
    state = g1motor.MotorState.decode(REPLY_FAULT)
    assert state.fault() and state.error == 0x8
    assert state.mode == g1motor.MODE_IDLE


def test_bad_frames_rejected():
    """!
    @brief Corrupt or short frames decode to None.
    """
    broken = bytearray(REPLY_ROBOT)
    broken[10] ^= 1
    assert g1motor.MotorState.decode(bytes(broken)) is None
    assert g1motor.MotorState.decode(REPLY_ROBOT[:-1]) is None
    assert g1motor.MotorCmd.decode(CMD_IDLE[:-1] + b"\x00") is None


def test_find_reply_skips_echo_and_noise():
    """!
    @brief A reply is found after echo and noise.
    """
    assert g1motor.find_reply(CMD_IDLE + b"\xfc\xee\x00" + REPLY_BENCH).id == 4
    assert g1motor.find_reply(CMD_IDLE) is None


def test_field_range_checked():
    """!
    @brief Out-of-range fields and ids raise ValueError.
    """
    with pytest.raises(ValueError):
        g1motor.MotorCmd(id=4, kp=40000).encode()
    with pytest.raises(ValueError):
        g1motor.MotorCmd(id=15).encode()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
