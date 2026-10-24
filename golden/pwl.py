"""PLAN-style piecewise-linear (PWL) integer sigmoid and tanh activations (§8.4).

Input `pre` is signed INT8 Q3.4 in [-128, 127], representing x in [-8.0, 7.9375].
- sigma_pwl(pre) -> UINT8 Q0.8 in [0, 255] (representing [0, 255/256]).
- tanh_pwl(pre)  -> INT8  Q0.7 in [-127, 127] (representing [-127/128, +127/128]).
  Uses shared hardware identity: tanh(x) = 2 * sigma(2x) - 1.
"""


def _pwl_pos_q08(abs_q34: int) -> int:
    """Evaluate raw positive PLAN sigmoid in Q0.8 [128..256] for non-negative Q3.4 input."""
    ax = int(abs_q34)
    if ax >= 80:  # |x| >= 5.0
        return 256
    if ax >= 38:  # 2.375 <= |x| < 5.0: |x|/32 + 0.84375 -> (ax >> 1) + 216
        return (ax >> 1) + 216
    if ax >= 16:  # 1.0 <= |x| < 2.375: |x|/8 + 0.625 -> (ax << 1) + 160
        return (ax << 1) + 160
    # 0 <= |x| < 1.0: |x|/4 + 0.5 -> (ax << 2) + 128
    return (ax << 2) + 128


def sigma_pwl(pre_q34: int) -> int:
    """Bit-exact integer PLAN sigmoid on INT8 Q3.4 -> UINT8 Q0.8 [0..255]."""
    x = int(pre_q34)
    if x < -128:
        x = -128
    elif x > 127:
        x = 127

    ax = -x if x < 0 else x
    y_pos = _pwl_pos_q08(ax)
    if x < 0:
        y = 256 - y_pos
    else:
        y = 255 if y_pos > 255 else y_pos
    return int(y)


def tanh_pwl(pre_q34: int) -> int:
    """Bit-exact integer PLAN tanh on INT8 Q3.4 -> INT8 Q0.7 [-127..127] via 2*sigma(2x)-1."""
    x = int(pre_q34)
    if x < -128:
        x = -128
    elif x > 127:
        x = 127

    if x == 0:
        return 0

    ax2 = (-x if x < 0 else x) << 1
    y_pos = _pwl_pos_q08(ax2)
    mag = y_pos - 128
    if mag > 127:
        mag = 127
    return -mag if x < 0 else mag
