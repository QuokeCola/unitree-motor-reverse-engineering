"""Frames below were captured from a Unitree G1 arm bus on 2026-11-25."""
from g1_joint_motor_sdk import MODE_IDLE, MODE_RUN, MotorCmd, MotorState, crc32_core, find_reply

CMD_ZERO_TORQUE_ID0 = bytes.fromhex("feee90800000000000000000000000009b1e4106")
CMD_DAMPING_ID0 = bytes.fromhex("feee9080000000000000000000001900541ed614")
CMD_IDLE_ID0 = bytes.fromhex("feee000000000000000000000000000080462079")
CMD_HOLD_ID0 = bytes.fromhex("feee908000000000c8610000c2010a0071dc7c64")
REPLY_ID2 = bytes.fromhex("fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c")
REPLY_FAULT_ID4 = bytes.fromhex("fcee0426263000 00fdffc15d010008000000a80b0000a190371a".replace(" ", ""))
REPLY_BENCH_ID4 = bytes.fromhex("fcee041e1c30fbfff9ff1b08010000000000c00b000057221ecf")


def test_crc_matches_capture():
    assert crc32_core(CMD_ZERO_TORQUE_ID0[:16]).to_bytes(4, "little") == CMD_ZERO_TORQUE_ID0[16:]
    assert crc32_core(REPLY_ID2[2:22]).to_bytes(4, "little") == REPLY_ID2[22:]


def test_encode_matches_capture():
    assert MotorCmd(id=0).encode() == CMD_IDLE_ID0
    assert MotorCmd(id=0, mode=MODE_RUN, flag=True, byte3=0x80).encode() == CMD_ZERO_TORQUE_ID0
    assert MotorCmd(id=0, mode=MODE_RUN, flag=True, byte3=0x80, kd=25).encode() == CMD_DAMPING_ID0
    assert MotorCmd(id=0, mode=MODE_RUN, flag=True, byte3=0x80, pos=25032, kp=450, kd=10).encode() == CMD_HOLD_ID0


def test_command_round_trip():
    cmd = MotorCmd(id=6, mode=MODE_RUN, flag=True, byte3=0x90, tau=-12, vel=345, pos=-123456, kp=300, kd=80)
    assert MotorCmd.decode(cmd.encode()) == cmd


def test_decode_reply():
    s = MotorState.decode(REPLY_ID2)
    assert (s.id, s.mode, s.pos, s.vel, s.tau) == (2, 1, -180794, -8, -1)
    assert (s.temp_a, s.temp_b, s.voltage) == (36, 35, 48.5)
    b = MotorState.decode(REPLY_BENCH_ID4)
    assert (b.id, b.mode, b.pos, b.voltage) == (4, MODE_IDLE, 67611, 24.0)
    assert not b.fault and not s.fault


def test_fault_word():
    f = MotorState.decode(REPLY_FAULT_ID4)
    assert f.fault and f.error == 0x8 and f.mode == MODE_IDLE


def test_bad_frames_rejected():
    broken = bytearray(REPLY_ID2)
    broken[10] ^= 1
    assert MotorState.decode(bytes(broken)) is None
    assert MotorState.decode(REPLY_ID2[:-1]) is None
    assert MotorCmd.decode(CMD_IDLE_ID0[:-1] + b"\x00") is None


def test_find_reply_skips_echo_and_noise():
    assert find_reply(CMD_IDLE_ID0 + b"\xfc\xee\x00" + REPLY_BENCH_ID4).id == 4
    assert find_reply(CMD_IDLE_ID0) is None


def test_field_range_checked():
    import pytest
    with pytest.raises(ValueError):
        MotorCmd(id=4, kp=40000).encode()
    with pytest.raises(ValueError):
        MotorCmd(id=15).encode()
