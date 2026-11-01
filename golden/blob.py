"""QSPI Flash Model Blob packer and parser (§8.6)."""

from dataclasses import dataclass, field
import struct
import zlib
from typing import List, Dict
import numpy as np


BLOB_MAGIC = b"NILM"
DESC_MAGIC = b"NTPD"
BLOB_VERSION = 0x10


@dataclass
class PanelDescriptor:
    panel_id: int = 0
    F: int = 14
    H: int = 16
    NZ: int = 16
    NHEAD: int = 4
    cell_type: int = 0  # 0 = LSTM (4 gates), 1 = GRU (3 gates)
    wfmt: int = 0       # 0 = INT8, 1 = INT4
    nlayer: int = 1
    s_i: int = 7
    s_f: int = 7
    s_g: int = 7
    s_o: int = 7
    s_z: int = 7
    s_head: int = 4
    b_i: int = 0
    b_f: int = 0
    b_g: int = 0
    b_o: int = 0
    b_z: int = 0
    b_head: int = 0
    deb_n: int = 0
    deb_m: int = 1
    # Weight arrays:
    # lstm_bias: shape (H, num_gates) int16
    # lstm_w:    shape (H, num_gates, F + H) int8
    # dense_bias: shape (NZ,) int16
    # dense_w:    shape (NZ, H) int8
    # head_bias:  shape (NHEAD,) int16
    # head_theta: shape (NHEAD,) int16
    # head_w:     shape (NHEAD, NZ) int8
    lstm_bias: np.ndarray = field(default_factory=lambda: np.zeros((16, 4), dtype=np.int16))
    lstm_w: np.ndarray = field(default_factory=lambda: np.zeros((16, 4, 30), dtype=np.int8))
    dense_bias: np.ndarray = field(default_factory=lambda: np.zeros((16,), dtype=np.int16))
    dense_w: np.ndarray = field(default_factory=lambda: np.zeros((16, 16), dtype=np.int8))
    head_bias: np.ndarray = field(default_factory=lambda: np.zeros((4,), dtype=np.int16))
    head_theta: np.ndarray = field(default_factory=lambda: np.zeros((4,), dtype=np.int16))
    head_w: np.ndarray = field(default_factory=lambda: np.zeros((4, 16), dtype=np.int8))

    @property
    def num_gates(self) -> int:
        return 3 if self.cell_type == 1 else 4


def _pack_u24(val: int) -> bytes:
    v = int(val) & 0xFFFFFF
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])


def _unpack_u24(buf: bytes, offset: int) -> int:
    return buf[offset] | (buf[offset + 1] << 8) | (buf[offset + 2] << 16)


def pack_panel_weights(desc: PanelDescriptor) -> tuple[bytes, bytes, bytes]:
    """Serialize LSTM, Dense, and Head weight streams in hardware consumption order (§8.6)."""
    for name, maximum in (("F", 32), ("H", 16), ("NZ", 16), ("NHEAD", 8)):
        if not 1 <= getattr(desc, name) <= maximum:
            raise ValueError(f"{name} must be in 1..{maximum} for this RTL configuration")
    num_gates = desc.num_gates
    vec_len = desc.F + desc.H

    lstm_bytes = bytearray()
    for j in range(desc.H):
        for k in range(num_gates):
            b_val = int(desc.lstm_bias[j, k])
            lstm_bytes.extend(struct.pack("<h", b_val))
            for n in range(vec_len):
                w_val = int(desc.lstm_w[j, k, n])
                lstm_bytes.extend(struct.pack("<b", w_val))

    dense_bytes = bytearray()
    for l_idx in range(desc.NZ):
        b_val = int(desc.dense_bias[l_idx])
        dense_bytes.extend(struct.pack("<h", b_val))
        for j in range(desc.H):
            w_val = int(desc.dense_w[l_idx, j])
            dense_bytes.extend(struct.pack("<b", w_val))

    head_bytes = bytearray()
    for m in range(desc.NHEAD):
        b_val = int(desc.head_bias[m])
        t_val = int(desc.head_theta[m])
        head_bytes.extend(struct.pack("<hh", b_val, t_val))
        for l_idx in range(desc.NZ):
            w_val = int(desc.head_w[m, l_idx])
            head_bytes.extend(struct.pack("<b", w_val))

    return bytes(lstm_bytes), bytes(dense_bytes), bytes(head_bytes)


def pack_panel_descriptor_bytes(
    desc: PanelDescriptor,
    w_lstm_off: int,
    w_z_off: int,
    w_head_off: int,
    crc32_val: int,
) -> bytes:
    """Pack 64-byte Panel Descriptor (§8.6)."""
    hdr = bytearray(64)
    hdr[0:4] = DESC_MAGIC
    hdr[4] = desc.F & 0xFF
    hdr[5] = desc.H & 0xFF
    hdr[6] = desc.NZ & 0xFF
    hdr[7] = desc.NHEAD & 0xFF
    flags = (desc.cell_type & 0x1) | ((desc.wfmt & 0x1) << 1) | ((desc.nlayer & 0x3) << 2)
    hdr[8] = flags & 0xFF
    hdr[9] = (desc.s_i & 0x0F) | ((desc.b_i & 0x0F) << 4)
    hdr[10] = (desc.s_f & 0x0F) | ((desc.b_f & 0x0F) << 4)
    hdr[11] = (desc.s_g & 0x0F) | ((desc.b_g & 0x0F) << 4)
    hdr[12] = (desc.s_o & 0x0F) | ((desc.b_o & 0x0F) << 4)
    hdr[13] = (desc.s_z & 0x0F) | ((desc.b_z & 0x0F) << 4)
    hdr[14] = (desc.s_head & 0x0F) | ((desc.b_head & 0x0F) << 4)
    hdr[15] = (desc.deb_n & 0x0F) | ((desc.deb_m & 0x0F) << 4)
    hdr[16:19] = _pack_u24(w_lstm_off)
    hdr[19:22] = _pack_u24(w_z_off)
    hdr[22:25] = _pack_u24(w_head_off)
    struct.pack_into("<I", hdr, 28, crc32_val & 0xFFFFFFFF)
    return bytes(hdr)


@dataclass
class ModelBlob:
    panels: List[PanelDescriptor]
    raw_bytes: bytes
    desc_offsets: List[int]
    weight_offsets: List[Dict[str, int]]


def pack_model_blob(panels: List[PanelDescriptor]) -> ModelBlob:
    """Pack a complete multi-panel NILM model blob for QSPI flash."""
    npanel = len(panels)
    # 32-byte blob header followed by 64-byte descriptors, then weight streams
    header_size = 32
    desc_base = header_size
    weights_base = desc_base + 64 * npanel

    desc_offsets = []
    weight_offsets = []
    desc_blocks = []
    weight_blocks = bytearray()

    cur_w_off = weights_base
    for idx, p in enumerate(panels):
        d_off = desc_base + 64 * idx
        desc_offsets.append(d_off)

        lstm_b, dense_b, head_b = pack_panel_weights(p)
        w_lstm_off = cur_w_off
        w_z_off = w_lstm_off + len(lstm_b)
        w_head_off = w_z_off + len(dense_b)
        combined_w = lstm_b + dense_b + head_b
        crc = zlib.crc32(combined_w) & 0xFFFFFFFF

        desc_blocks.append(
            pack_panel_descriptor_bytes(p, w_lstm_off, w_z_off, w_head_off, crc)
        )
        weight_blocks.extend(combined_w)
        weight_offsets.append(
            {"lstm": w_lstm_off, "dense": w_z_off, "head": w_head_off, "crc32": crc}
        )
        cur_w_off += len(combined_w)

    hdr = bytearray(header_size)
    hdr[0:4] = BLOB_MAGIC
    hdr[4] = BLOB_VERSION
    hdr[5] = npanel & 0xFF
    for i in range(min(4, npanel)):
        hdr[8 + 3 * i : 8 + 3 * (i + 1)] = _pack_u24(desc_offsets[i])

    full_bytes = bytes(hdr) + b"".join(desc_blocks) + bytes(weight_blocks)
    return ModelBlob(
        panels=panels,
        raw_bytes=full_bytes,
        desc_offsets=desc_offsets,
        weight_offsets=weight_offsets,
    )


def unpack_panel_descriptor(blob_bytes: bytes, desc_offset: int) -> PanelDescriptor:
    """Unpack a PanelDescriptor and its weights from raw flash bytes."""
    hdr = blob_bytes[desc_offset : desc_offset + 64]
    if hdr[0:4] != DESC_MAGIC:
        raise ValueError(f"Bad descriptor magic: {hdr[0:4]!r}")
    F = hdr[4]
    H = hdr[5]
    NZ = hdr[6]
    NHEAD = hdr[7]
    flags = hdr[8]
    cell_type = flags & 0x1
    wfmt = (flags >> 1) & 0x1
    nlayer = (flags >> 2) & 0x3

    s_i, b_i = hdr[9] & 0x0F, (hdr[9] >> 4) & 0x0F
    s_f, b_f = hdr[10] & 0x0F, (hdr[10] >> 4) & 0x0F
    s_g, b_g = hdr[11] & 0x0F, (hdr[11] >> 4) & 0x0F
    s_o, b_o = hdr[12] & 0x0F, (hdr[12] >> 4) & 0x0F
    s_z, b_z = hdr[13] & 0x0F, (hdr[13] >> 4) & 0x0F
    s_head, b_head = hdr[14] & 0x0F, (hdr[14] >> 4) & 0x0F
    deb_n, deb_m = hdr[15] & 0x0F, (hdr[15] >> 4) & 0x0F

    w_lstm_off = _unpack_u24(hdr, 16)
    w_z_off = _unpack_u24(hdr, 19)
    w_head_off = _unpack_u24(hdr, 22)

    num_gates = 3 if cell_type == 1 else 4
    vec_len = F + H

    lstm_bias = np.zeros((H, num_gates), dtype=np.int16)
    lstm_w = np.zeros((H, num_gates, vec_len), dtype=np.int8)
    ptr = w_lstm_off
    for j in range(H):
        for k in range(num_gates):
            lstm_bias[j, k] = struct.unpack_from("<h", blob_bytes, ptr)[0]
            ptr += 2
            for n in range(vec_len):
                lstm_w[j, k, n] = struct.unpack_from("<b", blob_bytes, ptr)[0]
                ptr += 1

    dense_bias = np.zeros((NZ,), dtype=np.int16)
    dense_w = np.zeros((NZ, H), dtype=np.int8)
    ptr = w_z_off
    for l_idx in range(NZ):
        dense_bias[l_idx] = struct.unpack_from("<h", blob_bytes, ptr)[0]
        ptr += 2
        for j in range(H):
            dense_w[l_idx, j] = struct.unpack_from("<b", blob_bytes, ptr)[0]
            ptr += 1

    head_bias = np.zeros((NHEAD,), dtype=np.int16)
    head_theta = np.zeros((NHEAD,), dtype=np.int16)
    head_w = np.zeros((NHEAD, NZ), dtype=np.int8)
    ptr = w_head_off
    for m in range(NHEAD):
        b_val, t_val = struct.unpack_from("<hh", blob_bytes, ptr)
        head_bias[m] = b_val
        head_theta[m] = t_val
        ptr += 4
        for l_idx in range(NZ):
            head_w[m, l_idx] = struct.unpack_from("<b", blob_bytes, ptr)[0]
            ptr += 1

    return PanelDescriptor(
        F=F,
        H=H,
        NZ=NZ,
        NHEAD=NHEAD,
        cell_type=cell_type,
        wfmt=wfmt,
        nlayer=nlayer,
        s_i=s_i,
        s_f=s_f,
        s_g=s_g,
        s_o=s_o,
        s_z=s_z,
        s_head=s_head,
        b_i=b_i,
        b_f=b_f,
        b_g=b_g,
        b_o=b_o,
        b_z=b_z,
        b_head=b_head,
        deb_n=deb_n,
        deb_m=deb_m,
        lstm_bias=lstm_bias,
        lstm_w=lstm_w,
        dense_bias=dense_bias,
        dense_w=dense_w,
        head_bias=head_bias,
        head_theta=head_theta,
        head_w=head_w,
    )
