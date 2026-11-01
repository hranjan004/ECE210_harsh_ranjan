"""Direct signed MAC checks, independent of host and sequencer traffic."""
import cocotb
from cocotb.triggers import Timer
from golden.fixedpoint import sat24, requant8, requant16


async def tick(dut):
    dut.clk.value = 0
    await Timer(5, units="ns")
    dut.clk.value = 1
    await Timer(5, units="ns")


@cocotb.test()
async def signed_products_rounding_and_saturation(dut):
    for name in ("clk", "rst_n", "clr_sat", "load_bias", "mac_en", "bias_in",
                 "bias_shift", "w_in", "u_in", "req_shift"):
        getattr(dut, name).value = 0
    await tick(dut)
    dut.rst_n.value = 1
    # Every INT8 operand pair, including -128 * -128 and mixed signs.
    for w in range(-128, 128):
        for u in range(-128, 128):
            dut.load_bias.value = 1
            dut.mac_en.value = 0
            await tick(dut)
            dut.load_bias.value = 0
            dut.mac_en.value = 1
            dut.w_in.value = w
            dut.u_in.value = u
            await tick(dut)
            assert dut.acc_out.value.signed_integer == w * u, (w, u)
    # Bias overflow, accumulator overflow, sticky flag, and signed ties.
    for bias, shift, w, u in ((32767, 15, 127, 127), (-32768, 15, -128, 127),
                              (-3, 0, 0, 0), (3, 0, 0, 0), (-32768, 8, 127, 127)):
        dut.bias_in.value, dut.bias_shift.value = bias, shift
        dut.load_bias.value, dut.mac_en.value, dut.clr_sat.value = 1, 0, 1
        await tick(dut)
        acc, saturated = sat24(bias << shift)
        assert dut.acc_out.value.signed_integer == acc
        assert int(dut.sat_seen.value) == saturated
        dut.load_bias.value, dut.mac_en.value, dut.clr_sat.value = 0, 1, 0
        dut.w_in.value, dut.u_in.value = w, u
        await tick(dut)
        acc, overflow = sat24(acc + w * u)
        assert dut.acc_out.value.signed_integer == acc
        assert int(dut.sat_seen.value) == (saturated or overflow)
        dut.mac_en.value = 0
        for req_shift in range(16):
            dut.req_shift.value = req_shift
            await tick(dut)
            val8, sat8 = requant8(acc, req_shift)
            val16, sat16 = requant16(acc, req_shift)
            assert dut.req8_out.value.signed_integer == val8
            assert dut.req16_out.value.signed_integer == val16
            assert int(dut.req8_sat.value) == sat8
            assert int(dut.req16_sat.value) == sat16
        dut.clr_sat.value = 1
        await tick(dut)
        assert int(dut.sat_seen.value) == 0
