"""!
@file protocol.py
@brief Wire format of the Unitree G1 joint motor bus: frames and checksum.

Unofficial and reverse engineered. All values are raw counts as on the wire.
"""
from __future__ import annotations

import dataclasses
import struct

## Bus bit rate in baud (8N1).
BAUD = 6_000_000
## Header of a command frame.
CMD_HEADER = b"\xfe\xee"
## Header of a reply frame.
REPLY_HEADER = b"\xfc\xee"
## Length of a command frame in bytes.
CMD_LEN = 20
## Length of a reply frame in bytes.
REPLY_LEN = 26
## Id reserved as broadcast in the public GO-M8010-6 protocol; never polled.
BROADCAST_ID = 15

## Mode 0: commands ignored; joint passively braked.
MODE_IDLE = 0
## Mode 1: hybrid torque / position / velocity command.
MODE_RUN = 1
## Mode 7: motor restarts (about 0.3 s); clears a latched fault.
MODE_RESTART = 7


def crc32_core(data: bytes) -> int:
    """!
    @brief Checksum of the bus: CRC-32 over little-endian 32-bit words.

    Polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no reflection, no final
    XOR, each word fed most-significant bit first.

    @param data Bytes to check; the length must be a multiple of 4.
    @return The checksum.
    @exception ValueError The length is not a multiple of 4.
    """
    if len(data) % 4:
        raise ValueError("crc32_core needs a multiple of 4 bytes")
    crc = 0xFFFFFFFF
    for (word,) in struct.iter_unpack("<I", data):
        bit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
            if word & bit:
                crc ^= 0x04C11DB7
            bit >>= 1
    return crc


def _check_range(value: float, bits: int, name: str) -> int:
    """!
    @brief Rounds a value and checks that it fits a signed field.
    @param value Value to store.
    @param bits Width of the field.
    @param name Field name for the error message.
    @return The rounded value.
    @exception ValueError The value does not fit.
    """
    value = round(value)
    limit = 1 << (bits - 1)
    if not -limit <= value < limit:
        raise ValueError(f"{name} = {value} does not fit a {bits}-bit field")
    return value


@dataclasses.dataclass
class MotorCmd:
    """!
    @brief One command frame (controller to motor).

    The motor applies
    `tau + kp * (pos - measured) + kd * (vel - measured)`.
    """

    ## Motor id, 0-14.
    id: int
    ## MODE_IDLE, MODE_RUN or MODE_RESTART.
    mode: int = MODE_IDLE
    ## Feed-forward torque, int16.
    tau: int = 0
    ## Velocity target, int16.
    vel: int = 0
    ## Position target, int32.
    pos: int = 0
    ## Position gain, int16.
    kp: int = 0
    ## Damping gain, int16.
    kd: int = 0
    ## Top bit of the id/mode byte; not required.
    flag: bool = False
    ## Sequence byte; not required.
    byte3: int = 0

    def encode(self) -> bytes:
        """!
        @brief Builds the 20-byte frame, checksum included.
        @return The frame.
        @exception ValueError The id or a field does not fit.
        """
        if not 0 <= self.id < BROADCAST_ID:
            raise ValueError(f"motor id {self.id} out of range 0-14")
        head = self.id | ((self.mode & 0x07) << 4) | (0x80 if self.flag else 0)
        body = struct.pack(
            "<2sBBhhihh", CMD_HEADER, head, self.byte3 & 0xFF,
            _check_range(self.tau, 16, "tau"),
            _check_range(self.vel, 16, "vel"),
            _check_range(self.pos, 32, "pos"),
            _check_range(self.kp, 16, "kp"),
            _check_range(self.kd, 16, "kd"))
        return body + struct.pack("<I", crc32_core(body))

    @classmethod
    def decode(cls, frame: bytes) -> MotorCmd | None:
        """!
        @brief Parses a command frame.
        @param frame Exactly 20 bytes.
        @return The command, or None if the header or checksum is wrong.
        """
        if len(frame) != CMD_LEN or frame[:2] != CMD_HEADER:
            return None
        if struct.unpack_from("<I", frame, 16)[0] != crc32_core(frame[:16]):
            return None
        head, byte3, tau, vel, pos, kp, kd = struct.unpack_from(
            "<BBhhihh", frame, 2)
        return cls(id=head & 0x0F, mode=(head >> 4) & 0x07, tau=tau, vel=vel,
                   pos=pos, kp=kp, kd=kd, flag=bool(head & 0x80), byte3=byte3)


@dataclasses.dataclass
class MotorState:
    """!
    @brief One reply frame (motor to controller).
    """

    ## Motor id.
    id: int
    ## Mode the motor is actually in.
    mode: int
    ## Net applied torque, int16.
    tau: int
    ## Velocity; 1 count = 128 position counts per second.
    vel: int
    ## Position; 32768 counts per rotor turn.
    pos: int
    ## Winding temperature, deg C.
    temp_coil: int
    ## Shell temperature, deg C.
    temp_shell: int
    ## Bus voltage, 0.5 V per count.
    voltage_raw: int
    ## Status word; 0 in normal operation.
    error: int
    ## Unknown.
    word18: int
    ## Unknown.
    byte20: int
    ## Unknown.
    byte21: int
    ## The frame as received.
    raw: bytes = b""

    @property
    def voltage(self) -> float:
        """!
        @brief Bus voltage.
        @return Volts.
        """
        return self.voltage_raw * 0.5

    @property
    def fault(self) -> bool:
        """!
        @brief Whether the motor reports a fault.
        @return True if the status word is non-zero.
        """
        return self.error != 0

    @classmethod
    def decode(cls, frame: bytes) -> MotorState | None:
        """!
        @brief Parses a reply frame.
        @param frame Exactly 26 bytes.
        @return The state, or None if the header or checksum is wrong.
        """
        if len(frame) != REPLY_LEN or frame[:2] != REPLY_HEADER:
            return None
        # The reply checksum covers bytes 2-21; the header is left out.
        if struct.unpack_from("<I", frame, 22)[0] != crc32_core(frame[2:22]):
            return None
        (head, temp_coil, temp_shell, voltage_raw, tau, vel, pos, error,
         word18, byte20, byte21) = struct.unpack_from("<BBBBhhiIHBB", frame, 2)
        return cls(id=head & 0x0F, mode=(head >> 4) & 0x07, tau=tau, vel=vel,
                   pos=pos, temp_coil=temp_coil, temp_shell=temp_shell,
                   voltage_raw=voltage_raw, error=error, word18=word18,
                   byte20=byte20, byte21=byte21, raw=bytes(frame))


def find_reply(buffer: bytes) -> MotorState | None:
    """!
    @brief Finds the first valid reply in a receive buffer.

    Skips echoed command bytes and noise.

    @param buffer Bytes received so far.
    @return The reply, or None if none is complete and valid.
    """
    at = buffer.find(REPLY_HEADER)
    while at >= 0:
        state = MotorState.decode(buffer[at:at + REPLY_LEN])
        if state:
            return state
        at = buffer.find(REPLY_HEADER, at + 1)
    return None
