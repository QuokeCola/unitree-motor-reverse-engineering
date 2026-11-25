"""Frame building and parsing for the G1 arm motor bus. See protocol/g1-arm-bus.md."""
import struct

CMD_LEN, REPLY_LEN = 20, 26


def crc32_core(data: bytes) -> int:
    """Unitree crc32_core over little-endian 32-bit words."""
    crc = 0xFFFFFFFF
    for (word,) in struct.iter_unpack("<I", data):
        bit = 1 << 31
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7 if crc & 0x80000000 else crc << 1) & 0xFFFFFFFF
            if word & bit:
                crc ^= 0x04C11DB7
            bit >>= 1
    return crc


def build_command(motor_id: int, mode: int = 0, flag: bool = False, byte3: int = 0,
                  tau: int = 0, vel: int = 0, pos: int = 0, kp: int = 0, kd: int = 0) -> bytes:
    """20-byte command. mode 0 = idle, 1 = running. All values are raw counts."""
    head = (motor_id & 0x0F) | ((mode & 0x07) << 4) | (0x80 if flag else 0)
    body = struct.pack("<2sBBhhihh", b"\xfe\xee", head, byte3, tau, vel, pos, kp, kd)
    return body + struct.pack("<I", crc32_core(body))


def parse_reply(frame: bytes) -> dict | None:
    """Decode a 26-byte reply; None if the header or checksum is wrong."""
    if len(frame) != REPLY_LEN or frame[:2] != b"\xfc\xee":
        return None
    if struct.unpack_from("<I", frame, 22)[0] != crc32_core(frame[2:22]):
        return None
    head, ta, tb, b5, tau, vel, pos, word14, u18, b20, b21 = struct.unpack_from("<BBBBhhiIHBB", frame, 2)
    return dict(id=head & 0x0F, mode=(head >> 4) & 0x07, temp_a=ta, temp_b=tb, b5=b5, tau=tau, vel=vel,
                pos=pos, word14=word14, u18=u18, b20=b20, b21=b21, raw=frame)
