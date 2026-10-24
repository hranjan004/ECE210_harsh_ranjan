"""Fixed-point arithmetic primitives for NILM-TT (round-half-up, saturation)."""

from typing import Tuple


def sat8(val: int) -> Tuple[int, bool]:
    """Clamp integer to signed INT8 [-128, 127] and return (value, saturated)."""
    v = int(val)
    if v > 127:
        return 127, True
    if v < -128:
        return -128, True
    return v, False


def sat16(val: int) -> Tuple[int, bool]:
    """Clamp integer to signed INT16 [-32768, 32767] and return (value, saturated)."""
    v = int(val)
    if v > 32767:
        return 32767, True
    if v < -32768:
        return -32768, True
    return v, False


def sat24(val: int) -> Tuple[int, bool]:
    """Clamp integer to signed INT24 [-8388608, 8388607] and return (value, saturated)."""
    v = int(val)
    if v > 8388607:
        return 8388607, True
    if v < -8388608:
        return -8388608, True
    return v, False


def rshift_round(val: int, shift: int) -> int:
    """Arithmetic right shift with round-half-up: (val + (1 << (s - 1))) >> s."""
    v = int(val)
    s = int(shift) & 0xF
    if s == 0:
        return v
    return (v + (1 << (s - 1))) >> s


def requant8(acc: int, shift: int) -> Tuple[int, bool]:
    """Requantize 24-bit accumulator to signed INT8 via round-half-up shift and saturation."""
    shifted = rshift_round(acc, shift)
    return sat8(shifted)


def requant16(acc: int, shift: int) -> Tuple[int, bool]:
    """Requantize 24-bit accumulator to signed INT16 via round-half-up shift and saturation."""
    shifted = rshift_round(acc, shift)
    return sat16(shifted)


def sign_extend(val: int, bits: int) -> int:
    """Convert unsigned bit pattern of width `bits` to signed Python int."""
    mask = (1 << bits) - 1
    v = int(val) & mask
    if v & (1 << (bits - 1)):
        v -= 1 << bits
    return v


def to_u8(val: int) -> int:
    return int(val) & 0xFF


def to_u16(val: int) -> int:
    return int(val) & 0xFFFF


def crc8_maxim(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    """Standard MSB-first CRC-8 (polynomial 0x07) used for optional SPI frame checks."""
    crc = init & 0xFF
    for b in data:
        crc ^= (b & 0xFF)
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc
