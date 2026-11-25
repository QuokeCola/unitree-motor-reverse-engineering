"""Wire format of the G1 joint motor bus: frame layout, checksum, encode and decode.

Everything here works in raw counts, exactly as they appear on the wire.
See units.py for conversion to physical units (those scales are assumptions).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

BAUD = 6_000_000
CMD_HEADER = b"\xfe\xee"
REPLY_HEADER = b"\xfc\xee"
CMD_LEN = 20
REPLY_LEN = 26
BROADCAST_ID = 15      # broadcast in the public GO-M8010-6 protocol; never polled by this SDK

MODE_IDLE = 0          # motor answers but applies no torque
MODE_RUN = 1           # motor runs its torque / position / velocity loop


def crc32_core(data: bytes) -> int:
    """Checksum used by the bus (same routine as Unitree's public A1/B1 SDK).

    CRC-32, polynomial 0x04C11DB7, init 0xFFFFFFFF, no reflection, no final XOR, fed
    32-bit little-endian words most-significant bit first. len(data) must be a multiple of 4.
    """
    if len(data) % 4:
        raise ValueError("crc32_core needs a multiple of 4 bytes")
    crc = 0xFFFFFFFF
    for (word,) in struct.iter_unpack("<I", data):
        bit = 1 << 31
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7 if crc & 0x80000000 else crc << 1) & 0xFFFFFFFF
            if word & bit:
                crc ^= 0x04C11DB7
            bit >>= 1
    return crc


@dataclass
class MotorCmd:
    """One command frame. All values are raw counts.

    The motor applies  tau + kp * (pos - measured_pos) + kd * (vel - measured_vel)
    (the usual Unitree hybrid command; confirmed here for pos/kp, kd and tau feed-forward).
    """
    id: int
    mode: int = MODE_IDLE
    tau: int = 0           # feed-forward torque, int16
    vel: int = 0           # velocity target, int16
    pos: int = 0           # position target, int32
    kp: int = 0            # position gain, int16
    kd: int = 0            # velocity (damping) gain, int16
    flag: bool = False     # top bit of the id/mode byte; the G1 controller sets it once polling is running
    byte3: int = 0         # see MotorBus: the controller toggles this between 0x80 and 0x90

    def encode(self) -> bytes:
        if not 0 <= self.id < BROADCAST_ID:
            raise ValueError(f"motor id {self.id} out of range 0-14")
        head = self.id | ((self.mode & 0x07) << 4) | (0x80 if self.flag else 0)
        body = struct.pack("<2sBBhhihh", CMD_HEADER, head, self.byte3 & 0xFF,
                           _i16(self.tau), _i16(self.vel), _i32(self.pos), _i16(self.kp), _i16(self.kd))
        return body + struct.pack("<I", crc32_core(body))

    @classmethod
    def decode(cls, frame: bytes) -> "MotorCmd | None":
        """Parse a captured command frame; None if the header or checksum is wrong."""
        if len(frame) != CMD_LEN or frame[:2] != CMD_HEADER:
            return None
        if struct.unpack_from("<I", frame, 16)[0] != crc32_core(frame[:16]):
            return None
        head, byte3, tau, vel, pos, kp, kd = struct.unpack_from("<BBhhihh", frame, 2)
        return cls(id=head & 0x0F, mode=(head >> 4) & 0x07, tau=tau, vel=vel, pos=pos, kp=kp, kd=kd,
                   flag=bool(head & 0x80), byte3=byte3)


@dataclass
class MotorState:
    """One reply frame, raw counts."""
    id: int
    mode: int              # mode the motor is actually in
    tau: int               # torque, int16
    vel: int               # velocity, int16
    pos: int               # position, int32 (multi-turn)
    temp_a: int            # looks like a temperature in deg C
    temp_b: int            # second temperature, usually 1-2 below temp_a
    voltage_raw: int       # bus voltage, 0.5 V per count
    error: int             # status / fault word: 0 in normal running. 0x8 seen latched on three bench
                           # motors after a hard stop from top speed; they then refuse run mode.
                           # 0x00040000 and 0x8000 patterns seen briefly during robot boot.
    word18: int            # unknown, about 2,600-3,050
    byte20: int            # unknown, 0 so far
    byte21: int            # unknown; changes every frame on two of the robot's arm motors
    raw: bytes = b""

    @property
    def fault(self) -> bool:
        """True if the status word is non-zero."""
        return self.error != 0

    @property
    def voltage(self) -> float:
        """Bus voltage in volts (checked at 24 V and about 48 V)."""
        return self.voltage_raw * 0.5

    @classmethod
    def decode(cls, frame: bytes) -> "MotorState | None":
        """Parse a reply frame; None if the header or checksum is wrong."""
        if len(frame) != REPLY_LEN or frame[:2] != REPLY_HEADER:
            return None
        if struct.unpack_from("<I", frame, 22)[0] != crc32_core(frame[2:22]):   # header is not covered
            return None
        head, ta, tb, volt, tau, vel, pos, error, word18, b20, b21 = struct.unpack_from("<BBBBhhiIHBB", frame, 2)
        return cls(id=head & 0x0F, mode=(head >> 4) & 0x07, tau=tau, vel=vel, pos=pos, temp_a=ta, temp_b=tb,
                   voltage_raw=volt, error=error, word18=word18, byte20=b20, byte21=b21, raw=bytes(frame))


def find_reply(buffer: bytes) -> "MotorState | None":
    """First valid reply frame inside a receive buffer (skips echo and noise)."""
    at = buffer.find(REPLY_HEADER)
    while at >= 0:
        state = MotorState.decode(buffer[at:at + REPLY_LEN])
        if state:
            return state
        at = buffer.find(REPLY_HEADER, at + 1)
    return None


def _i16(x: float) -> int:
    x = round(x)
    if not -32768 <= x <= 32767:
        raise ValueError(f"value {x} does not fit a 16-bit field")
    return x


def _i32(x: float) -> int:
    x = round(x)
    if not -2**31 <= x < 2**31:
        raise ValueError(f"value {x} does not fit a 32-bit field")
    return x
