"""Host SPI Master Driver for NILM-TT cocotb tests (§7.2, §11.3)."""

import struct
from typing import List
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from golden.fixedpoint import crc8_maxim, sign_extend


def start_chip_clock(dut, period_ns: int = 20):
    """Start dut.clk compatibility wrapper supporting both cocotb 1.9 (`units`) and 2.0 (`unit`)."""
    try:
        clock = Clock(dut.clk, period_ns, unit="ns")
    except TypeError:
        clock = Clock(dut.clk, period_ns, units="ns")
    cocotb.start_soon(clock.start())


class HostSpiDriver:
    """Bit-accurate Mode-0 Host SPI driver operating on ui_in[2:0] and uo_out[0]."""

    def __init__(self, dut, half_sck_clks: int = 4):
        self.dut = dut
        self.half_sck_clks = max(4, int(half_sck_clks))
        self.crc_enabled = False
        self.bus_req = 0
        self.run_pin = 0
        self.dbg_sel = 0

    def _drive_ui(self, cs_n: int, sck: int, mosi: int):
        val = (
            (int(cs_n) & 1)
            | ((int(sck) & 1) << 1)
            | ((int(mosi) & 1) << 2)
            | ((int(self.bus_req) & 1) << 3)
            | ((int(self.run_pin) & 1) << 4)
            | ((int(self.dbg_sel) & 0x7) << 5)
        )
        self.dut.ui_in.value = val

    async def reset_chip(self, cycles: int = 10):
        self.bus_req = 0
        self.run_pin = 0
        self.dbg_sel = 0
        self.dut.ena.value = 1
        self._drive_ui(cs_n=1, sck=0, mosi=0)
        self.dut.uio_in.value = 0
        self.dut.rst_n.value = 0
        await ClockCycles(self.dut.clk, cycles)
        assert int(self.dut.uio_oe.value) == 0, "uio_oe must be 0x00 during reset"
        self.dut.rst_n.value = 1
        await ClockCycles(self.dut.clk, 6)

    async def xfer_frame(
        self,
        tx_bytes: bytes | List[int],
        append_crc: bool | None = None,
        abort_after_bits: int | None = None,
    ) -> bytes:
        """Execute one Mode-0 SPI frame with CS_N low."""
        payload = bytearray(int(b) & 0xFF for b in tx_bytes)
        use_crc = self.crc_enabled if append_crc is None else append_crc
        if use_crc:
            payload.append(crc8_maxim(bytes(payload)))

        rx_bytes = bytearray()
        self._drive_ui(cs_n=0, sck=0, mosi=0)
        await ClockCycles(self.dut.clk, self.half_sck_clks)

        total_bits = 0
        for b_out in payload:
            b_in = 0
            for bit_pos in range(7, -1, -1):
                if abort_after_bits is not None and total_bits >= abort_after_bits:
                    self._drive_ui(cs_n=1, sck=0, mosi=0)
                    await ClockCycles(self.dut.clk, self.half_sck_clks + 4)
                    return bytes(rx_bytes)

                mosi = (b_out >> bit_pos) & 1
                self._drive_ui(cs_n=0, sck=0, mosi=mosi)
                await ClockCycles(self.dut.clk, self.half_sck_clks)

                # Rising edge of HOST_SCK: slave samples MOSI; master samples MISO
                self._drive_ui(cs_n=0, sck=1, mosi=mosi)
                await ClockCycles(self.dut.clk, self.half_sck_clks)
                miso = int(self.dut.uo_out.value) & 1
                b_in = (b_in << 1) | miso
                total_bits += 1

            # Falling edge after 8th bit
            self._drive_ui(cs_n=0, sck=0, mosi=0)
            await ClockCycles(self.dut.clk, self.half_sck_clks)
            rx_bytes.append(b_in)

        self._drive_ui(cs_n=1, sck=0, mosi=0)
        await ClockCycles(self.dut.clk, self.half_sck_clks + 4)
        return bytes(rx_bytes)

    async def read_id(self) -> bytes:
        rx = await self.xfer_frame([0xA5, 0x00, 0x00, 0x00, 0x00], append_crc=False)
        return rx[2:5]

    async def wr_reg(self, addr: int, data: int, bad_crc: bool = False):
        frame = bytes([0x01, addr & 0xFF, data & 0xFF])
        if self.crc_enabled:
            c = crc8_maxim(frame) ^ (0xFF if bad_crc else 0x00)
            await self.xfer_frame(frame + bytes([c]), append_crc=False)
        else:
            await self.xfer_frame(frame, append_crc=False)

    async def rd_reg(self, addr: int) -> int:
        rx = await self.xfer_frame([0x02, addr & 0xFF, 0x00, 0x00], append_crc=False)
        return rx[3]

    async def rd_reg_burst(self, start_addr: int, count: int) -> bytes:
        tx = [0x03, start_addr & 0xFF, count & 0xFF, 0x00] + [0x00] * count
        rx = await self.xfer_frame(tx, append_crc=False)
        return rx[4 : 4 + count]

    async def load_x(self, panel: int, features: List[int] | bytes):
        f_bytes = [int(x) & 0xFF for x in features]
        tx = [0x10, panel & 0xFF, len(f_bytes) & 0xFF] + f_bytes
        await self.xfer_frame(tx)

    async def load_ctx(self, panel: int, ctx_100b: bytes):
        assert len(ctx_100b) == 100
        tx = bytes([0x11, panel & 0xFF]) + ctx_100b
        await self.xfer_frame(tx)

    async def read_ctx(self, panel: int) -> bytes:
        tx = [0x12, panel & 0xFF, 0x00] + [0x00] * 100
        rx = await self.xfer_frame(tx, append_crc=False)
        return rx[3:103]

    async def run_panel(self, panel: int):
        await self.xfer_frame([0x20, panel & 0xFF])

    async def run_all(self):
        await self.xfer_frame([0x21])

    async def rd_result(self, panel: int) -> int:
        rx = await self.xfer_frame([0x30, panel & 0xFF, 0x00, 0x00, 0x00], append_crc=False)
        return rx[3] | (rx[4] << 8)

    async def rd_logit(self, panel: int, head: int) -> int:
        rx = await self.xfer_frame([0x31, panel & 0xFF, head & 0xFF, 0x00, 0x00, 0x00], append_crc=False)
        u16 = rx[4] | (rx[5] << 8)
        return sign_extend(u16, 16)

    async def ctx_reset(self, panel: int = 0):
        await self.xfer_frame([0x40, panel & 0xFF])

    async def qspi_init(self):
        await self.xfer_frame([0x50])

    async def bus_release(self):
        await self.xfer_frame([0x60])

    async def bus_acquire(self):
        await self.xfer_frame([0x61])

    async def mem_rd(self, dev: int, addr: int, length: int) -> bytes:
        tx = [
            0x70,
            dev & 0xFF,
            (addr >> 16) & 0xFF,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            length & 0xFF,
        ]
        await self.xfer_frame(tx, append_crc=False)
        await self.wait_idle()
        return await self.rd_reg_burst(0x40, length)

    async def mem_wr(self, dev: int, addr: int, data: bytes):
        tx = [
            0x71,
            dev & 0xFF,
            (addr >> 16) & 0xFF,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            len(data) & 0xFF,
        ] + list(data)
        await self.xfer_frame(tx)
        await self.wait_idle()

    async def step_unit(self):
        await self.xfer_frame([0x80])

    async def wait_idle(self, max_cycles: int = 150000) -> int:
        """Wait until BUSY (uo_out[2]) deasserts and return elapsed clk cycles."""
        waited = 0
        while waited < max_cycles:
            await ClockCycles(self.dut.clk, 10)
            waited += 10
            if ((int(self.dut.uo_out.value) >> 2) & 1) == 0:
                return waited
        raise TimeoutError(f"Chip remained BUSY after {max_cycles} cycles")
