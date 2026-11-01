"""Unit tests for the NILM-TT Python golden model (Layer L1)."""

import math
import pytest
import numpy as np

from golden.fixedpoint import sat8, sat16, sat24, rshift_round, requant8, requant16, crc8_maxim
from golden.pwl import sigma_pwl, tanh_pwl
from golden.blob import PanelDescriptor, pack_model_blob, unpack_panel_descriptor
from golden.nilm_golden import PanelContext, NilmGolden, lstm_cell_update


def test_pwl_exhaustive_256():
    """Verify all 256 INT8 codes for sigma_pwl and tanh_pwl (§8.4, §11.3, §19)."""
    max_sig_err = 0.0
    max_tanh_err = 0.0
    prev_s = -1
    prev_t = -129
    for code in range(-128, 128):
        x_real = code / 16.0
        s_q08 = sigma_pwl(code)
        t_q07 = tanh_pwl(code)

        assert 0 <= s_q08 <= 255
        assert -127 <= t_q07 <= 127
        # Monotonicity
        assert s_q08 >= prev_s
        assert t_q07 >= prev_t
        prev_s = s_q08
        prev_t = t_q07

        sig_ref = 1.0 / (1.0 + math.exp(-x_real))
        tanh_ref = math.tanh(x_real)
        max_sig_err = max(max_sig_err, abs(s_q08 / 256.0 - sig_ref))
        max_tanh_err = max(max_tanh_err, abs(t_q07 / 128.0 - tanh_ref))

    # PLAN approximation error must be <= 0.02 (~0.019 reported in §8.4)
    assert max_sig_err < 0.020
    assert max_tanh_err < 0.040
    assert sigma_pwl(0) == 128
    assert tanh_pwl(0) == 0


def test_fixedpoint_rounding_and_saturation():
    assert rshift_round(3, 1) == 2
    assert rshift_round(-3, 1) == -1
    assert sat8(200) == (127, True)
    assert sat8(-200) == (-128, True)
    assert sat16(40000) == (32767, True)
    assert sat16(-40000) == (-32768, True)
    assert sat24(1 << 24)[1] is True
    assert crc8_maxim(b"\x01\x02\x03") == crc8_maxim(b"\x01\x02\x03")


def test_blob_pack_unpack_roundtrip():
    rng = np.random.default_rng(42)
    p0 = PanelDescriptor(
        panel_id=0,
        F=14,
        H=8,
        NZ=4,
        NHEAD=4,
        lstm_bias=rng.integers(-500, 500, size=(8, 4), dtype=np.int16),
        lstm_w=rng.integers(-64, 64, size=(8, 4, 22), dtype=np.int8),
        dense_bias=rng.integers(-200, 200, size=(4,), dtype=np.int16),
        dense_w=rng.integers(-64, 64, size=(4, 8), dtype=np.int8),
        head_bias=rng.integers(-200, 200, size=(4,), dtype=np.int16),
        head_theta=np.array([-32, 0, 64, 128], dtype=np.int16),
        head_w=rng.integers(-64, 64, size=(4, 4), dtype=np.int8),
    )
    blob = pack_model_blob([p0])
    assert blob.raw_bytes[0:4] == b"NILM"
    unpacked = unpack_panel_descriptor(blob.raw_bytes, blob.desc_offsets[0])
    assert unpacked.F == 14
    assert unpacked.H == 8
    assert np.array_equal(unpacked.lstm_w, p0.lstm_w)
    assert np.array_equal(unpacked.dense_w, p0.dense_w)
    assert np.array_equal(unpacked.head_theta, p0.head_theta)


def test_golden_step_and_debounce():
    p = PanelDescriptor(
        panel_id=0,
        F=4,
        H=4,
        NZ=4,
        NHEAD=2,
        deb_n=2,
        deb_m=3,
        lstm_bias=np.zeros((4, 4), dtype=np.int16),
        lstm_w=np.ones((4, 4, 8), dtype=np.int8) * 16,
        dense_bias=np.zeros((4,), dtype=np.int16),
        dense_w=np.ones((4, 4), dtype=np.int8) * 32,
        head_bias=np.array([100, -100], dtype=np.int16),
        head_theta=np.array([10, 10], dtype=np.int16),
        head_w=np.ones((2, 4), dtype=np.int8) * 16,
    )
    ctx = PanelContext()
    x = np.array([16, 16, 16, 16], dtype=np.int8)

    # Step 1: raw head 0 is 1, but 2-of-3 debounce requires 2 hits
    ctx1, logits1, bits1, _ = NilmGolden.step(p, x, ctx, deb_en=True)
    assert logits1[0] >= 10
    assert (bits1 & 1) == 0

    # Step 2: second consecutive hit -> 2-of-3 debounce activates head 0
    ctx2, logits2, bits2, _ = NilmGolden.step(p, x, ctx1, deb_en=True)
    assert (bits2 & 1) == 1

    # Context roundtrip
    raw_ctx = ctx2.to_bytes()
    assert len(raw_ctx) == 100
    ctx2_rt = PanelContext.from_bytes(raw_ctx)
    assert np.array_equal(ctx2_rt.h, ctx2.h)
    assert np.array_equal(ctx2_rt.c, ctx2.c)


@pytest.mark.parametrize("field,value", [("F", 0), ("F", 33), ("H", 0), ("H", 17),
                                        ("NZ", 0), ("NZ", 17), ("NHEAD", 0), ("NHEAD", 9)])
def test_blob_rejects_unsupported_dimensions(field, value):
    panel = PanelDescriptor()
    setattr(panel, field, value)
    with pytest.raises(ValueError, match=field):
        pack_model_blob([panel])


def test_default_blob_fits_rtl():
    panel = PanelDescriptor()
    blob = pack_model_blob([panel])
    decoded = unpack_panel_descriptor(blob.raw_bytes, blob.desc_offsets[0])
    assert decoded.H == 16
    assert decoded.lstm_w.shape == (16, 4, 30)
