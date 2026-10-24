"""Behavioural model for AP Memory APS6404L-3SQR QSPI PSRAM and shared Pmod harness (§8.2, §11.3).

Supports:
- 66h / 99h Reset sequence
- EBh Quad Read (8 SCK cmd + 6 SCK addr + 6 SCK wait, then 2 SCK/byte data)
- 38h Quad Write (8 SCK cmd + 6 SCK addr + 0 wait, then 2 SCK/byte data)
- Protocol checkers:
  * At most one CS low at a time (CS0, CS1, CS2)
  * SD lines never driven by chip during Flash/PSRAM read data phase
  * PSRAM CE#-low burst duration monitor (max_ce_low_clks / tCEM compliance)
  * Programmable read output delay in clk cycles (to test RD_LAT / RD_HALF)
"""

from collections import deque
import cocotb
from cocotb.triggers import RisingEdge, ReadWrite
from .qspi_flash import QspiFlashModel


class PsramAps6404Model:
    def __init__(self, size_bytes: int = 16384):
        self.mem = bytearray(size_bytes)
        self.reset_enabled = False
        self.reset_count = 0
        self.read_bursts = 0
        self.write_bursts = 0
        self.max_ce_low_clks = 0
        self._reset_fsm()

    def _reset_fsm(self):
        self.phase = "CMD"
        self.sck_cnt = 0
        self.cmd_reg = 0
        self.addr_reg = 0
        self.hi_nib = 0
        self.driving = False
        self.out_nib = 0

    def read_bytes(self, addr: int, length: int) -> bytes:
        return bytes(self.mem[addr : addr + length])

    def write_bytes(self, addr: int, data: bytes):
        self.mem[addr : addr + len(data)] = data

    def on_cs_high(self):
        if self.phase == "CMD" and self.sck_cnt == 8:
            if self.cmd_reg == 0x66:
                self.reset_enabled = True
            elif self.cmd_reg == 0x99 and self.reset_enabled:
                self.reset_count += 1
                self.reset_enabled = False
        self._reset_fsm()

    def on_sck_rise(self, sd_in: int):
        if self.phase == "CMD":
            mosi = sd_in & 0x1
            self.cmd_reg = ((self.cmd_reg << 1) | mosi) & 0xFF
            self.sck_cnt += 1
            if self.sck_cnt == 8:
                if self.cmd_reg in (0xEB, 0x38):
                    self.phase = "ADDR"
                    self.sck_cnt = 0
                    self.addr_reg = 0
        elif self.phase == "ADDR":
            self.addr_reg = ((self.addr_reg << 4) | (sd_in & 0xF)) & 0xFFFFFF
            self.sck_cnt += 1
            if self.sck_cnt == 6:
                if self.cmd_reg == 0xEB:
                    self.phase = "WAIT"
                    self.sck_cnt = 0
                else:
                    self.phase = "WRITE"
                    self.sck_cnt = 0
                    self.write_bursts += 1
        elif self.phase == "WAIT":
            self.sck_cnt += 1
        elif self.phase == "WRITE":
            if (self.sck_cnt & 1) == 0:
                self.hi_nib = sd_in & 0xF
            else:
                b_val = ((self.hi_nib << 4) | (sd_in & 0xF)) & 0xFF
                self.mem[self.addr_reg % len(self.mem)] = b_val
                self.addr_reg = (self.addr_reg + 1) & 0xFFFFFF
            self.sck_cnt += 1

    def on_sck_fall(self):
        if self.phase == "WAIT" and self.sck_cnt == 6:
            self.phase = "READ"
            self.sck_cnt = 0
            self.read_bursts += 1
            b_val = self.mem[self.addr_reg % len(self.mem)]
            self.out_nib = (b_val >> 4) & 0xF
            self.driving = True
        elif self.phase == "READ":
            self.sck_cnt += 1
            b_val = self.mem[self.addr_reg % len(self.mem)]
            if (self.sck_cnt & 1) == 1:
                self.out_nib = b_val & 0xF
            else:
                self.addr_reg = (self.addr_reg + 1) & 0xFFFFFF
                b_next = self.mem[self.addr_reg % len(self.mem)]
                self.out_nib = (b_next >> 4) & 0xF
            self.driving = True


class QspiPmodHarness:
    """Pin-accurate QSPI Pmod monitor & responder attached to dut.uio_*."""

    def __init__(self, dut, flash: QspiFlashModel | None = None, psram: PsramAps6404Model | None = None):
        self.dut = dut
        self.flash = flash if flash is not None else QspiFlashModel()
        self.psram = psram if psram is not None else PsramAps6404Model()
        self.output_delay_clks = 0
        self.protocol_errors = []
        self._delay_q = deque()
        self._running = False

    @staticmethod
    def _nib_to_uio(nib: int) -> int:
        # uio[1]=SD0, [2]=SD1, [4]=SD2, [5]=SD3
        n = int(nib) & 0xF
        return ((n & 0x1) << 1) | ((n & 0x2) << 1) | ((n & 0x4) << 2) | ((n & 0x8) << 2)

    @staticmethod
    def _uio_to_nib(uio_val: int) -> int:
        v = int(uio_val) & 0xFF
        return ((v >> 1) & 0x1) | ((v >> 1) & 0x2) | ((v >> 2) & 0x4) | ((v >> 2) & 0x8)

    def start(self):
        if not self._running:
            self._running = True
            cocotb.start_soon(self._run())

    async def _run(self):
        prev_sck = 0
        prev_cs0 = 1
        prev_cs1 = 1
        ce1_low_clks = 0

        while True:
            await RisingEdge(self.dut.clk)
            await ReadWrite()
            try:
                uio_out = int(self.dut.uio_out.value)
                uio_oe = int(self.dut.uio_oe.value)
            except ValueError:
                continue

            if uio_oe == 0:
                cs0 = 1
                cs1 = 1
                cs2 = 1
                sck = 0
            else:
                cs0 = (uio_out >> 0) & 1
                sck = (uio_out >> 3) & 1
                cs1 = (uio_out >> 6) & 1
                cs2 = (uio_out >> 7) & 1

            # Formal invariant check: at most one CS low at a time
            low_count = (1 - cs0) + (1 - cs1) + (1 - cs2)
            if low_count > 1:
                self.protocol_errors.append(f"Multiple CS low simultaneously: cs0={cs0}, cs1={cs1}, cs2={cs2}")

            # Track PSRAM CE# low duration in clk cycles
            if cs1 == 0:
                ce1_low_clks += 1
                if ce1_low_clks > self.psram.max_ce_low_clks:
                    self.psram.max_ce_low_clks = ce1_low_clks
            else:
                ce1_low_clks = 0

            # Handle CS rising edges
            if prev_cs0 == 0 and cs0 == 1:
                self.flash.on_cs_high()
            if prev_cs1 == 0 and cs1 == 1:
                self.psram.on_cs_high()

            sd_in = self._uio_to_nib(uio_out)

            # SCK rising edge
            if prev_sck == 0 and sck == 1:
                if cs0 == 0:
                    self.flash.on_sck_rise(sd_in)
                if cs1 == 0:
                    self.psram.on_sck_rise(sd_in)

            # SCK falling edge
            if prev_sck == 1 and sck == 0:
                if cs0 == 0:
                    self.flash.on_sck_fall()
                if cs1 == 0:
                    self.psram.on_sck_fall()

            # Check SD bus contention during read data phase
            sd_oe_bits = uio_oe & 0x36
            if (self.flash.driving or self.psram.driving) and sd_oe_bits != 0:
                self.protocol_errors.append(f"Bus contention: chip driving SD (uio_oe=0x{uio_oe:02x}) during read")

            # Determine active slave nibble output and apply programmable output delay
            raw_nib = 0
            if cs0 == 0 and self.flash.driving:
                raw_nib = self.flash.out_nib
            elif cs1 == 0 and self.psram.driving:
                raw_nib = self.psram.out_nib

            self._delay_q.append(raw_nib)
            while len(self._delay_q) > self.output_delay_clks + 1:
                self._delay_q.popleft()
            delayed_nib = self._delay_q[0]

            self.dut.uio_in.value = self._nib_to_uio(delayed_nib)

            prev_sck = sck
            prev_cs0 = cs0
            prev_cs1 = cs1
