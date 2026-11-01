"""Direct LSTM/GRU shared-multiplier checks against integer golden functions."""
import random
import cocotb
from golden.nilm_golden import lstm_cell_update, gru_cell_update
from golden.fixedpoint import sat8, rshift_round
from test_mac import tick


@cocotb.test()
async def signed_cell_edges_and_random(dut):
    for name in ("clk", "rst_n", "cell_en", "cell_step", "cell_type", "a_i", "a_f",
                 "a_g", "a_o", "c_old", "h_old"):
        getattr(dut, name).value = 0
    await tick(dut)
    dut.rst_n.value = 1
    rng = random.Random(808)
    vectors = [(255, 255, g, 255, c, h) for g in (-128, 127)
               for c in (-32768, 32767) for h in (-128, 127)]
    vectors += [tuple(rng.randint(lo, hi) for lo, hi in
                      ((0, 255), (0, 255), (-128, 127), (0, 255), (-32768, 32767), (-128, 127)))
                for _ in range(512)]
    for mode in (0, 1):
        dut.cell_type.value = mode
        for ai, af, ag, ao, c, h in vectors:
            for name, value in zip(("a_i", "a_f", "a_g", "a_o", "c_old", "h_old"),
                                   (ai, af, ag, ao, c, h)):
                getattr(dut, name).value = value
            dut.cell_en.value, dut.cell_step.value = 0, 3
            await tick(dut)
            assert dut.gru_r_h.value.signed_integer == sat8(rshift_round(af * h, 8))[0]
            dut.cell_en.value = 1
            saturated = False
            for step in (0, 1, 2):
                dut.cell_step.value = step
                await tick(dut)
                saturated |= bool(dut.cell_sat.value)
            if mode:
                expected_c, expected_h, expected_sat = gru_cell_update(ai, ag, h)
            else:
                expected_c, expected_h, _, expected_sat = lstm_cell_update(ai, af, ag, ao, c)
            assert dut.c_new.value.signed_integer == expected_c
            assert dut.h_new.value.signed_integer == expected_h
            assert saturated == expected_sat
