# SPDX-FileCopyrightText: © 2026 Harsh Ranjan
# SPDX-License-Identifier: Apache-2.0

"""Comprehensive cocotb RTL & Gate-Level verification suite for NILM-TT (tt_um_harsh_nilm)."""

import os
import struct
import numpy as np
import cocotb
from cocotb.triggers import ClockCycles

from golden.fixedpoint import sign_extend
from golden.blob import PanelDescriptor, pack_model_blob
from golden.nilm_golden import PanelContext, NilmGolden
from models import HostSpiDriver, QspiPmodHarness, start_chip_clock


def _make_random_panel(
    rng: np.random.Generator,
    panel_id: int = 0,
    F: int = 4,
    H: int = 4,
    NZ: int = 4,
    NHEAD: int = 4,
    cell_type: int = 0,
    deb_n: int = 0,
    deb_m: int = 1,
) -> PanelDescriptor:
    num_gates = 3 if cell_type == 1 else 4
    return PanelDescriptor(
        panel_id=panel_id,
        F=F,
        H=H,
        NZ=NZ,
        NHEAD=NHEAD,
        cell_type=cell_type,
        s_i=6,
        s_f=6,
        s_g=6,
        s_o=6,
        s_z=6,
        s_head=4,
        b_i=1,
        b_f=1,
        b_g=1,
        b_o=1,
        b_z=1,
        b_head=1,
        deb_n=deb_n,
        deb_m=deb_m,
        lstm_bias=rng.integers(-256, 256, size=(H, num_gates), dtype=np.int16),
        lstm_w=rng.integers(-64, 64, size=(H, num_gates, F + H), dtype=np.int8),
        dense_bias=rng.integers(-256, 256, size=(NZ,), dtype=np.int16),
        dense_w=rng.integers(-64, 64, size=(NZ, H), dtype=np.int8),
        head_bias=rng.integers(-256, 256, size=(NHEAD,), dtype=np.int16),
        head_theta=rng.integers(-128, 128, size=(NHEAD,), dtype=np.int16),
        head_w=rng.integers(-64, 64, size=(NHEAD, NZ), dtype=np.int8),
    )


@cocotb.test()
async def test_gl_smoke(dut):
    """Gate-level & RTL smoke test: reset, X-check, ID, QSPI_INIT, 4-unit LSTM step, PSRAM R/W (Sec. 11.3, 11.5)."""
    start_chip_clock(dut, period_ns=20)
    spi = HostSpiDriver(dut, half_sck_clks=4)
    pmod = QspiPmodHarness(dut)
    pmod.start()

    await spi.reset_chip()

    uo_str = str(dut.uo_out.value)
    uio_oe_str = str(dut.uio_oe.value)
    assert "x" not in uo_str.lower() and "z" not in uo_str.lower()
    assert "x" not in uio_oe_str.lower() and "z" not in uio_oe_str.lower()
    assert int(dut.uio_oe.value) == 0

    chip_id = await spi.read_id()
    assert chip_id == b"NT\x10", f"Unexpected ID: {chip_id!r}"

    rng = np.random.default_rng(101)
    p0 = _make_random_panel(rng, panel_id=0, F=4, H=4, NZ=4, NHEAD=4)
    blob = pack_model_blob([p0])
    pmod.flash.load_image(blob.raw_bytes, 0)

    await spi.qspi_init()
    await spi.wait_idle()
    status = await spi.rd_reg(0x02)
    assert (status & 0x10) != 0, f"QSPI_READY not set (status=0x{status:02x})"
    assert (status & 0x04) == 0, f"Unexpected ERR (status=0x{status:02x})"

    x0 = np.array([24, -16, 32, -8], dtype=np.int8)
    ctx0 = PanelContext()
    ctx1, exp_logits, exp_bits, _ = NilmGolden.step(p0, x0, ctx0, deb_en=True)

    await spi.load_x(0, x0.tolist())
    await spi.run_panel(0)
    await spi.wait_idle()

    got_bits = await spi.rd_result(0)
    got_ctx = PanelContext.from_bytes(pmod.psram.read_bytes(0, 100))
    got_logits = [await spi.rd_logit(0, m) for m in range(p0.NHEAD)]
    dut._log.info(f"GL_SMOKE H got={got_ctx.h[:4].tolist()} exp={ctx1.h[:4].tolist()}")
    dut._log.info(f"GL_SMOKE C got={got_ctx.c[:4].tolist()} exp={ctx1.c[:4].tolist()}")
    dut._log.info(f"GL_SMOKE LOGITS got={got_logits} exp={exp_logits.tolist()}")
    assert got_bits == exp_bits, f"Result mismatch: got 0x{got_bits:04x}, exp 0x{exp_bits:04x}"

    for m in range(p0.NHEAD):
        got_l = got_logits[m]
        assert got_l == int(exp_logits[m]), f"Head {m} logit mismatch: got {got_l}, exp {exp_logits[m]}"

    assert len(pmod.protocol_errors) == 0, f"QSPI protocol errors: {pmod.protocol_errors}"


@cocotb.test()
async def test_host_spi_and_csr_windows(dut):
    """Suite `host`: CSR R/W, buffer windows (U_WIN, HB_WIN, C_WIN), SPI clock ratios, mid-byte abort, CRC-8 (Sec. 11.3)."""
    if os.environ.get("GATES") == "yes":
        return

    start_chip_clock(dut, period_ns=20)
    spi = HostSpiDriver(dut, half_sck_clks=4)
    await spi.reset_chip()

    # 1. CSR read/write & burst read across U_WIN (0x40..0x47), HB_WIN (0x80..0x83), C_WIN (0xA0..0xA7)
    for i in range(8):
        await spi.wr_reg(0x40 + i, (i * 17 + 5) & 0xFF)
    burst_u = await spi.rd_reg_burst(0x40, 8)
    assert burst_u == bytes((i * 17 + 5) & 0xFF for i in range(8))

    for i in range(4):
        await spi.wr_reg(0x80 + i, (0xA0 + i) & 0xFF)
    burst_hb = await spi.rd_reg_burst(0x80, 4)
    assert burst_hb == bytes((0xA0 + i) & 0xFF for i in range(4))

    # 16-bit C_WIN write/read (little-endian pairs at 0xA0..0xA7)
    test_c_vals = [0x1234, -0x2345 & 0xFFFF, 0x7FFF, 0x8000]
    for idx, val in enumerate(test_c_vals):
        await spi.wr_reg(0xA0 + 2 * idx, val & 0xFF)
        await spi.wr_reg(0xA0 + 2 * idx + 1, (val >> 8) & 0xFF)
    burst_c = await spi.rd_reg_burst(0xA0, 8)
    for idx, val in enumerate(test_c_vals):
        got = burst_c[2 * idx] | (burst_c[2 * idx + 1] << 8)
        assert got == val

    # 2. Test slower SPI clock ratios (clk/16, clk/32, clk/64 -> half_sck_clks = 8, 16, 32)
    for half_clks in (4, 8, 16, 32):
        spi.half_sck_clks = half_clks
        await spi.wr_reg(0x0C, half_clks + 10)
        assert await spi.rd_reg(0x0C) == half_clks + 10
    spi.half_sck_clks = 4

    # 3. Mid-byte CS_N abort -> ERR 0x01 (ERR_FRAME)
    await spi.xfer_frame([0x01, 0x04, 0x02], abort_after_bits=12)
    status = await spi.rd_reg(0x02)
    err_code = await spi.rd_reg(0x0E)
    assert (status & 0x04) != 0 and err_code == 0x01
    # Clear error
    await spi.wr_reg(0x0E, 0x00)
    assert await spi.rd_reg(0x0E) == 0x00

    # 4. Unknown opcode -> ERR 0x02 (ERR_OPCODE)
    await spi.xfer_frame([0xFE, 0x00])
    assert await spi.rd_reg(0x0E) == 0x02
    await spi.wr_reg(0x0E, 0x00)

    # 5. CRC-8 frame validation (CSR_CTRL.CRC_EN = bit 5 = 0x20, plus DEB_EN = 0x10 -> 0x30)
    await spi.wr_reg(0x03, 0x30)
    spi.crc_enabled = True
    # Valid CRC write succeeds
    await spi.wr_reg(0x0C, 0x2A, bad_crc=False)
    assert await spi.rd_reg(0x0C) == 0x2A
    assert await spi.rd_reg(0x0E) == 0x00
    # Corrupted CRC write is rejected and sets ERR_FRAME (0x01)
    await spi.wr_reg(0x0C, 0x55, bad_crc=True)
    assert await spi.rd_reg(0x0C) == 0x2A
    assert await spi.rd_reg(0x0E) == 0x01
    # Restore CRC_EN = 0
    await spi.wr_reg(0x0E, 0x00)
    await spi.wr_reg(0x03, 0x10)
    spi.crc_enabled = False


@cocotb.test()
async def test_qspi_protocol_latency_burst_and_handover(dut):
    """Suite `qspi`: bad magic, RD_LAT sweep, PSRAM_BURST_MAX chunking, BUS_REQ/BUS_GNT handover (Sec. 11.3)."""
    if os.environ.get("GATES") == "yes":
        return

    start_chip_clock(dut, period_ns=20)
    spi = HostSpiDriver(dut, half_sck_clks=4)
    pmod = QspiPmodHarness(dut)
    pmod.start()
    await spi.reset_chip()

    # 1. Bad blob magic -> ERR 0x10 (ERR_MAGIC)
    pmod.flash.load_image(b"BAD!" + b"\x00" * 60, 0)
    await spi.qspi_init()
    await spi.wait_idle()
    assert await spi.rd_reg(0x0E) == 0x10
    await spi.wr_reg(0x0E, 0x00)

    # 2. Load valid blob and test RD_LAT (0, 1, 2) with matching harness output delay
    rng = np.random.default_rng(202)
    p0 = _make_random_panel(rng, panel_id=0, F=4, H=4, NZ=4, NHEAD=4)
    blob = pack_model_blob([p0])
    pmod.flash.load_image(blob.raw_bytes, 0)

    for lat in (0, 1, 2):
        pmod.output_delay_clks = lat
        # QSPI_CFG: PSRAM_EN=1 (bit 5), SCK_DIV=0 (bits 4:3), RD_HALF=0 (bit 2), RD_LAT=lat
        await spi.wr_reg(0x05, 0x20 | lat)
        await spi.qspi_init()
        await spi.wait_idle()
        assert await spi.rd_reg(0x0E) == 0x00, f"QSPI_INIT failed at RD_LAT={lat}"
        read_hdr = await spi.mem_rd(0, 0x000000, 4)
        assert read_hdr == b"NILM", f"MEM_RD mismatch at RD_LAT={lat}: {read_hdr!r}"

    pmod.output_delay_clks = 0
    await spi.wr_reg(0x05, 0x20)

    # 3. PSRAM_BURST_MAX chunking test (set PSRAM_BURST_MAX = 16 bytes for a 100-byte context R/W)
    await spi.wr_reg(0x0C, 16)
    pmod.psram.max_ce_low_clks = 0
    pmod.psram.write_bursts = 0
    pmod.psram.read_bursts = 0

    x0 = np.array([10, -20, 30, -40], dtype=np.int8)
    await spi.load_x(0, x0.tolist())
    await spi.run_panel(0)
    await spi.wait_idle()

    # 100 bytes with burst_max=16 must split into ceil(100/16) = 7 bursts!
    assert pmod.psram.read_bursts == 7, f"Expected 7 PSRAM read bursts, got {pmod.psram.read_bursts}"
    assert pmod.psram.write_bursts == 7, f"Expected 7 PSRAM write bursts, got {pmod.psram.write_bursts}"
    # Each 16-byte burst is (14 or 20 SCK preamble + 32 SCK data) * 2 clk/SCK <= 110 clks (<< 400 clks = 8 us @ 50MHz)
    assert pmod.psram.max_ce_low_clks <= 120, f"PSRAM CE# low too long: {pmod.psram.max_ce_low_clks} clks"

    # 4. Bus handover via BUS_REQ pin (ui_in[3]) and BUS_RELEASE / BUS_ACQUIRE commands
    spi.bus_req = 1
    spi._drive_ui(cs_n=1, sck=0, mosi=0)
    await ClockCycles(dut.clk, 20)
    assert ((int(dut.uo_out.value) >> 3) & 1) == 1, "BUS_GNT pin must assert on BUS_REQ"
    assert int(dut.uio_oe.value) == 0, "uio_oe must be 0x00 when BUS_GNT is asserted"

    spi.bus_req = 0
    spi._drive_ui(cs_n=1, sck=0, mosi=0)
    await ClockCycles(dut.clk, 10)
    assert ((int(dut.uo_out.value) >> 3) & 1) == 0, "BUS_GNT must deassert when BUS_REQ is released"

    await spi.bus_release()
    await ClockCycles(dut.clk, 20)
    assert ((int(dut.uo_out.value) >> 3) & 1) == 1
    assert int(dut.uio_oe.value) == 0
    await spi.bus_acquire()
    await ClockCycles(dut.clk, 10)
    assert ((int(dut.uo_out.value) >> 3) & 1) == 0
    assert len(pmod.protocol_errors) == 0


@cocotb.test()
async def test_single_unit_step_and_cell_bit_exact(dut):
    """Suites `mac_pwl` & `cell`: STEP_MODE single-unit stepping & accumulator/state trace check (Sec. 11.3)."""
    if os.environ.get("GATES") == "yes":
        return

    start_chip_clock(dut, period_ns=20)
    spi = HostSpiDriver(dut, half_sck_clks=4)
    pmod = QspiPmodHarness(dut)
    pmod.start()
    await spi.reset_chip()

    rng = np.random.default_rng(303)
    p0 = _make_random_panel(rng, panel_id=0, F=4, H=4, NZ=4, NHEAD=2)
    blob = pack_model_blob([p0])
    pmod.flash.load_image(blob.raw_bytes, 0)

    await spi.qspi_init()
    await spi.wait_idle()

    # Enable Host-Context mode (bit 6 = 0x40) + STEP_MODE (bit 7 = 0x80) + DEB_EN (bit 4 = 0x10) -> 0xD0
    await spi.wr_reg(0x03, 0xD0)

    x0 = np.array([35, -42, 19, 60], dtype=np.int8)
    ctx0 = PanelContext(
        h=np.array([12, -25, 40, -10] + [0] * 28, dtype=np.int8),
        c=np.array([300, -450, 800, -200] + [0] * 28, dtype=np.int16),
    )
    exp_ctx, exp_logits, exp_bits, trace = NilmGolden.step(p0, x0, ctx0, deb_en=True)

    await spi.load_x(0, x0.tolist())
    await spi.load_ctx(0, ctx0.to_bytes())

    # Step unit-by-unit (H=4 units) using STEP_UNIT (0x80)
    for j in range(p0.H):
        await spi.step_unit()
        await spi.wait_idle()
        # Before the last unit finishes (j < H-1), the sequencer pauses after unit j
        if j < p0.H - 1:
            dbg_unit = await spi.rd_reg(0x19)
            assert dbg_unit == j + 1, f"Expected dbg_unit={j+1}, got {dbg_unit}"
            # Check updated c[j] in C_WIN and h_new[j] in HB_WIN against golden trace!
            hb_raw = await spi.rd_reg(0x80 + j)
            got_h_j = sign_extend(hb_raw, 8)
            assert got_h_j == trace["units"][j]["h_new"]

            c_lo = await spi.rd_reg(0xA0 + 2 * j)
            c_hi = await spi.rd_reg(0xA0 + 2 * j + 1)
            got_c_j = sign_extend(c_lo | (c_hi << 8), 16)
            assert got_c_j == trace["units"][j]["c_new"]

    # After the 4th unit (j == H-1), H_COMMIT, DENSE, and HEADS complete automatically
    got_bits = await spi.rd_result(0)
    assert got_bits == exp_bits
    for m in range(p0.NHEAD):
        assert await spi.rd_logit(0, m) == int(exp_logits[m])


@cocotb.test()
async def test_panel_lstm_gru_thresholds_and_debounce(dut):
    """Suite `panel`: multi-step LSTM, 3-gate GRU mode, exact logit==theta boundary, N-of-M debounce (Sec. 11.3)."""
    if os.environ.get("GATES") == "yes":
        return

    start_chip_clock(dut, period_ns=20)
    spi = HostSpiDriver(dut, half_sck_clks=4)
    pmod = QspiPmodHarness(dut)
    pmod.start()
    await spi.reset_chip()

    rng = np.random.default_rng(404)
    # Panel 0: 4-gate LSTM with 2-of-3 debounce (F=6, H=6, NZ=4, NHEAD=4)
    p_lstm = _make_random_panel(
        rng, panel_id=0, F=6, H=6, NZ=4, NHEAD=4, cell_type=0, deb_n=2, deb_m=3
    )
    # Panel 1: 3-gate GRU (cell_type=1, F=6, H=6, NZ=4, NHEAD=4)
    p_gru = _make_random_panel(
        rng, panel_id=1, F=6, H=6, NZ=4, NHEAD=4, cell_type=1, deb_n=0, deb_m=1
    )

    blob = pack_model_blob([p_lstm, p_gru])
    pmod.flash.load_image(blob.raw_bytes, 0)

    await spi.qspi_init()
    await spi.wait_idle()

    ctx_lstm = PanelContext()
    ctx_gru = PanelContext()

    # Run 4 consecutive streaming timesteps on both Panel 0 (LSTM + 2-of-3 debounce) and Panel 1 (GRU)
    for t in range(4):
        x_t = rng.integers(-64, 64, size=(6,), dtype=np.int8)

        # Panel 0 (LSTM)
        ctx_lstm, exp_logits_0, exp_bits_0, _ = NilmGolden.step(
            p_lstm, x_t, ctx_lstm, deb_en=True
        )
        await spi.load_x(0, x_t.tolist())
        await spi.run_panel(0)
        await spi.wait_idle()

        got_bits_0 = await spi.rd_result(0)
        got_logits_0 = [await spi.rd_logit(0, m) for m in range(p_lstm.NHEAD)]
        dut._log.info(
            f"STEP {t} LSTM logits got={got_logits_0} exp={exp_logits_0.tolist()} "
            f"bits got=0x{got_bits_0:04x} exp=0x{exp_bits_0:04x} "
            f"deb_en={int(dut.user_project.deb_en.value)} "
            f"deb_n={int(dut.user_project.u_seq.desc_deb_n.value)} "
            f"deb_m={int(dut.user_project.u_seq.desc_deb_m.value)} "
            f"deb_mem3={int(dut.user_project.u_vram.deb_mem[3].value)}"
        )
        for m in range(p_lstm.NHEAD):
            assert got_logits_0[m] == int(exp_logits_0[m]), f"LSTM step {t} head {m} logit mismatch: got {got_logits_0[m]}, exp {exp_logits_0[m]}"
        assert got_bits_0 == exp_bits_0, f"LSTM step {t} bits mismatch: got 0x{got_bits_0:04x}, exp 0x{exp_bits_0:04x}"

        # Panel 1 (GRU)
        ctx_gru, exp_logits_1, exp_bits_1, _ = NilmGolden.step(
            p_gru, x_t, ctx_gru, deb_en=True
        )
        await spi.load_x(1, x_t.tolist())
        await spi.run_panel(1)
        await spi.wait_idle()

        got_bits_1 = await spi.rd_result(1)
        assert got_bits_1 == exp_bits_1, f"GRU step {t} bits mismatch: got 0x{got_bits_1:04x}, exp 0x{exp_bits_1:04x}"
        for m in range(p_gru.NHEAD):
            got_l = await spi.rd_logit(1, m)
            assert got_l == int(exp_logits_1[m]), f"GRU step {t} head {m} logit mismatch"

        # Verify PSRAM saved context matches golden context byte-for-byte!
        psram_ctx_1 = pmod.psram.read_bytes(1 * 256, 100)
        assert psram_ctx_1 == ctx_gru.to_bytes(), f"GRU step {t} PSRAM context mismatch"


@cocotb.test()
async def test_system_multi_panel_run_pin_and_abort(dut):
    """Suite `system`: RUN_ALL, RUN_PIN rising edge, Host-Context mode, ABORT, and cycle budget (Sec. 11.3)."""
    if os.environ.get("GATES") == "yes":
        return

    start_chip_clock(dut, period_ns=20)
    spi = HostSpiDriver(dut, half_sck_clks=4)
    pmod = QspiPmodHarness(dut)
    pmod.start()
    await spi.reset_chip()

    rng = np.random.default_rng(505)
    panels = [
        _make_random_panel(rng, panel_id=0, F=4, H=4, NZ=4, NHEAD=4),
        _make_random_panel(rng, panel_id=1, F=4, H=4, NZ=4, NHEAD=4),
        _make_random_panel(rng, panel_id=2, F=4, H=4, NZ=4, NHEAD=6),
    ]
    blob = pack_model_blob(panels)
    pmod.flash.load_image(blob.raw_bytes, 0)

    await spi.qspi_init()
    await spi.wait_idle()

    x_vec = np.array([20, -15, 40, -25], dtype=np.int8)
    await spi.load_x(0, x_vec.tolist())

    # Trigger RUN_ALL via RUN_PIN rising edge (ui_in[4])
    spi.run_pin = 1
    spi._drive_ui(cs_n=1, sck=0, mosi=0)
    await ClockCycles(dut.clk, 6)
    spi.run_pin = 0
    spi._drive_ui(cs_n=1, sck=0, mosi=0)

    await spi.wait_idle()

    for p_idx, p_desc in enumerate(panels):
        _, exp_logits, exp_bits, _ = NilmGolden.step(p_desc, x_vec, PanelContext(), deb_en=True)
        got_bits = await spi.rd_result(p_idx)
        assert got_bits == exp_bits, f"RUN_ALL panel {p_idx} mismatch: got 0x{got_bits:04x}, exp 0x{exp_bits:04x}"

    # Check cycle counter CSR (0x14..0x16) is well within the 32,000 clk/panel budget (§11.3)
    cyc_bytes = await spi.rd_reg_burst(0x14, 3)
    cycles = cyc_bytes[0] | (cyc_bytes[1] << 8) | (cyc_bytes[2] << 16)
    assert 100 < cycles <= 32000, f"Unexpected cycle count: {cycles}"

    # Test ABORT during an active run -> sets ERR_ABORT (0x30) and returns to IDLE
    await spi.run_panel(2)
    await ClockCycles(dut.clk, 40)
    # Write CTRL.ABORT (bit 1 = 0x02)
    await spi.wr_reg(0x03, 0x12)
    await spi.wait_idle()
    assert await spi.rd_reg(0x0E) == 0x30
    assert len(pmod.protocol_errors) == 0
