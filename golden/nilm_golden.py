"""Bit-exact integer golden model for NILM-TT (§5.2, §6.2, §8.3–8.7, §11.2)."""

from dataclasses import dataclass, field
import struct
from typing import Dict, Any, List, Tuple
import numpy as np

from .fixedpoint import sat8, sat16, sat24, rshift_round, requant8, requant16
from .pwl import sigma_pwl, tanh_pwl
from .blob import PanelDescriptor


@dataclass
class PanelContext:
    """Per-panel recurrent state (h: 32 x int8, c: 32 x int16, deb_hist: 16 x uint8 4-bit)."""
    h: np.ndarray = field(default_factory=lambda: np.zeros((32,), dtype=np.int8))
    c: np.ndarray = field(default_factory=lambda: np.zeros((32,), dtype=np.int16))
    deb_hist: np.ndarray = field(default_factory=lambda: np.zeros((16,), dtype=np.uint8))

    def copy(self) -> "PanelContext":
        return PanelContext(
            h=self.h.copy(),
            c=self.c.copy(),
            deb_hist=self.deb_hist.copy(),
        )

    def to_bytes(self) -> bytes:
        """Pack into the 100-byte PSRAM/host context format (32B h + 64B c + 4B deb)."""
        buf = bytearray(100)
        for i in range(32):
            buf[i] = int(self.h[i]) & 0xFF
        for i in range(32):
            struct.pack_into("<h", buf, 32 + 2 * i, int(self.c[i]))
        # Pack low 2 bits of deb_hist[0..15] across 4 bytes (or 4 bits for heads 0..7 in 4 bytes)
        # Wait: let's store 4-bit history for heads 0..7 in bytes 96..99 (2 nibbles/byte)
        for m in range(8):
            b_idx = 96 + (m >> 1)
            nib = int(self.deb_hist[m]) & 0x0F
            if (m & 1) == 0:
                buf[b_idx] = (buf[b_idx] & 0xF0) | nib
            else:
                buf[b_idx] = (buf[b_idx] & 0x0F) | (nib << 4)
        return bytes(buf)

    @classmethod
    def from_bytes(cls, data: bytes, prev_deb_hi: np.ndarray | None = None) -> "PanelContext":
        if len(data) < 100:
            raise ValueError(f"Expected 100 bytes of context, got {len(data)}")
        h = np.zeros((32,), dtype=np.int8)
        c = np.zeros((32,), dtype=np.int16)
        deb_hist = np.zeros((16,), dtype=np.uint8)
        for i in range(32):
            h[i] = struct.unpack_from("<b", data, i)[0]
        for i in range(32):
            c[i] = struct.unpack_from("<h", data, 32 + 2 * i)[0]
        for m in range(8):
            b_val = data[96 + (m >> 1)]
            nib = (b_val & 0x0F) if (m & 1) == 0 else ((b_val >> 4) & 0x0F)
            deb_hist[m] = nib
        if prev_deb_hi is not None:
            for m in range(8, 16):
                deb_hist[m] = int(prev_deb_hi[m]) & 0x0F
        return cls(h=h, c=c, deb_hist=deb_hist)


def mac_dot_int24(
    bias_i16: int,
    bias_shift: int,
    weights_i8: np.ndarray,
    operands_i8: np.ndarray,
) -> Tuple[int, bool]:
    """Compute signed 24-bit MAC dot product with per-step saturation tracking."""
    sat_seen = False
    acc, s = sat24(int(bias_i16) << (int(bias_shift) & 0xF))
    sat_seen = sat_seen or s
    for w, u in zip(weights_i8, operands_i8):
        prod = int(w) * int(u)
        acc, s = sat24(acc + prod)
        sat_seen = sat_seen or s
    return acc, sat_seen


def lstm_cell_update(
    a_i: int,
    a_f: int,
    a_g: int,
    a_o: int,
    c_old: int,
) -> Tuple[int, int, int, bool]:
    """Bit-exact LSTM cell update (§8.4).

    Returns (c_new_i16, h_new_i8, tc_i8, sat_seen).
    """
    sat_seen = False
    fc = rshift_round(int(a_f) * int(c_old), 8)
    ig = rshift_round(int(a_i) * int(a_g), 4)
    c_new, s1 = sat16(fc + ig)
    c_q34, s2 = sat8(rshift_round(c_new, 7))
    tc = tanh_pwl(c_q34)
    h_new, s3 = sat8(rshift_round(int(a_o) * int(tc), 8))
    sat_seen = s1 or s2 or s3
    return c_new, h_new, tc, sat_seen


def gru_cell_update(
    a_z: int,
    a_g: int,
    h_old: int,
) -> Tuple[int, int, bool]:
    """Bit-exact 3-gate GRU state update (§5.4, §8.6).

    h_new = sat8(rshift_round((256 - a_z) * h_old + a_z * a_g, 8))
    c_new = int16(h_new) << 4  (mirrored in c for inspection)
    """
    term_keep = (256 - int(a_z)) * int(h_old)
    term_new = int(a_z) * int(a_g)
    h_new, sat = sat8(rshift_round(term_keep + term_new, 8))
    c_new = int(h_new) << 4
    return c_new, h_new, sat


def popcount4(val: int) -> int:
    v = int(val) & 0x0F
    return (v & 1) + ((v >> 1) & 1) + ((v >> 2) & 1) + ((v >> 3) & 1)


class NilmGolden:
    """Reference integer model matching the tt_um_harsh_nilm RTL cycle-accurate dataflow."""

    @staticmethod
    def step(
        desc: PanelDescriptor,
        x_q: np.ndarray,
        ctx: PanelContext,
        deb_en: bool = True,
    ) -> Tuple[PanelContext, np.ndarray, int, Dict[str, Any]]:
        """Execute one streaming panel step.

        Returns:
            (new_ctx, logits_i16, result_bitmask_u16, trace_dict)
        """
        F = desc.F
        H = desc.H
        NZ = desc.NZ
        NHEAD = desc.NHEAD

        new_ctx = ctx.copy()
        sat_seen = False

        # Assemble operand vector u[0 .. F+H-1] = [x[0..F-1], h_prev[0..H-1]]
        u = np.zeros((F + H,), dtype=np.int8)
        u[0:F] = np.asarray(x_q[0:F], dtype=np.int8)
        u[F : F + H] = ctx.h[0:H]

        h_next = np.zeros((32,), dtype=np.int8)
        c_next = ctx.c.copy()
        unit_traces: List[Dict[str, Any]] = []

        if desc.cell_type == 0:
            # 4-gate LSTM: gates (i, f, g, o)
            shifts = [desc.s_i, desc.s_f, desc.s_g, desc.s_o]
            b_shifts = [desc.b_i, desc.b_f, desc.b_g, desc.b_o]
            for j in range(H):
                pre_gates = []
                act_gates = []
                acc_gates = []
                for k in range(4):
                    acc, s_mac = mac_dot_int24(
                        int(desc.lstm_bias[j, k]),
                        b_shifts[k],
                        desc.lstm_w[j, k, 0 : F + H],
                        u,
                    )
                    pre, s_req = requant8(acc, shifts[k])
                    sat_seen = sat_seen or s_mac or s_req
                    if k == 2:
                        act = tanh_pwl(pre)
                    else:
                        act = sigma_pwl(pre)
                    acc_gates.append(acc)
                    pre_gates.append(pre)
                    act_gates.append(act)

                c_j, h_j, tc_j, s_cell = lstm_cell_update(
                    act_gates[0],
                    act_gates[1],
                    act_gates[2],
                    act_gates[3],
                    int(ctx.c[j]),
                )
                sat_seen = sat_seen or s_cell
                c_next[j] = c_j
                h_next[j] = h_j
                unit_traces.append(
                    {
                        "unit": j,
                        "acc": acc_gates,
                        "pre": pre_gates,
                        "act": act_gates,
                        "c_new": c_j,
                        "tc": tc_j,
                        "h_new": h_j,
                    }
                )
        else:
            # 3-gate GRU: gates (z, r, g)
            shifts = [desc.s_i, desc.s_f, desc.s_g]
            b_shifts = [desc.b_i, desc.b_f, desc.b_g]
            for j in range(H):
                acc_gates = []
                pre_gates = []
                act_gates = []
                for k in range(3):
                    if k == 2:
                        # Gate recurrent inputs u[F..F+H-1] by reset gate a_r (act_gates[1])
                        u_gated = u.copy()
                        a_r = act_gates[1]
                        for n in range(F, F + H):
                            g_val, s_g_op = sat8(rshift_round(int(a_r) * int(u[n]), 8))
                            sat_seen = sat_seen or s_g_op
                            u_gated[n] = g_val
                        operands = u_gated
                    else:
                        operands = u

                    acc, s_mac = mac_dot_int24(
                        int(desc.lstm_bias[j, k]),
                        b_shifts[k],
                        desc.lstm_w[j, k, 0 : F + H],
                        operands,
                    )
                    pre, s_req = requant8(acc, shifts[k])
                    sat_seen = sat_seen or s_mac or s_req
                    act = tanh_pwl(pre) if k == 2 else sigma_pwl(pre)
                    acc_gates.append(acc)
                    pre_gates.append(pre)
                    act_gates.append(act)

                c_j, h_j, s_cell = gru_cell_update(
                    act_gates[0], act_gates[2], int(ctx.h[j])
                )
                sat_seen = sat_seen or s_cell
                c_next[j] = c_j
                h_next[j] = h_j
                unit_traces.append(
                    {
                        "unit": j,
                        "acc": acc_gates,
                        "pre": pre_gates,
                        "act": act_gates,
                        "c_new": c_j,
                        "h_new": h_j,
                    }
                )

        # H_COMMIT: active h becomes h_next[0..H-1]
        new_ctx.h = h_next.copy()
        new_ctx.c = c_next.copy()

        # DENSE ReLU layer: z[0..NZ-1] = ReLU(requant8(W_z * h_next + b_z))
        z_vec = np.zeros((NZ,), dtype=np.int8)
        for l_idx in range(NZ):
            acc, s_mac = mac_dot_int24(
                int(desc.dense_bias[l_idx]),
                desc.b_z,
                desc.dense_w[l_idx, 0:H],
                h_next[0:H],
            )
            pre_z, s_req = requant8(acc, desc.s_z)
            sat_seen = sat_seen or s_mac or s_req
            z_vec[l_idx] = pre_z if pre_z > 0 else 0

        # HEADS linear layer + threshold compare + optional N-of-M debounce
        logits = np.zeros((NHEAD,), dtype=np.int16)
        raw_bits = 0
        result_bits = 0
        m_win = max(1, min(4, int(desc.deb_m)))
        m_mask = (1 << m_win) - 1
        deb_n = int(desc.deb_n)

        for m in range(NHEAD):
            acc, s_mac = mac_dot_int24(
                int(desc.head_bias[m]),
                desc.b_head,
                desc.head_w[m, 0:NZ],
                z_vec[0:NZ],
            )
            logit_m, s_req = requant16(acc, desc.s_head)
            sat_seen = sat_seen or s_mac or s_req
            logits[m] = logit_m

            theta_m = int(desc.head_theta[m])
            raw_on = 1 if int(logit_m) >= theta_m else 0
            if raw_on:
                raw_bits |= 1 << m

            hist_new = ((int(ctx.deb_hist[m]) << 1) | raw_on) & 0x0F
            new_ctx.deb_hist[m] = hist_new

            if deb_en and deb_n > 0:
                cnt = popcount4(hist_new & m_mask)
                on_m = 1 if cnt >= deb_n else 0
            else:
                on_m = raw_on

            if on_m:
                result_bits |= 1 << m

        trace = {
            "units": unit_traces,
            "z": z_vec,
            "logits": logits,
            "raw_bits": raw_bits,
            "result_bits": result_bits,
            "sat_seen": sat_seen,
        }
        return new_ctx, logits, result_bits, trace
