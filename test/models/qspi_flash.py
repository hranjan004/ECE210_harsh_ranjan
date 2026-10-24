"""Behavioural model for Winbond W25Q128JV QSPI Flash (§8.2, §11.3).

Supports:
- 66h (Enable Reset) and 99h (Reset Device) in 1-bit SPI mode
- EBh (Fast Read Quad I/O, 1-4-4): 8 SCK cmd + 6 SCK addr + 2 SCK mode (Fh) + 4 SCK dummy,
  followed by continuous 2-SCK/byte streaming (MSB nibble first).
- Arbitrary SCK stall tolerance during read data phase.
"""


class QspiFlashModel:
    def __init__(self, size_bytes: int = 65536):
        self.mem = bytearray(b"\xFF" * size_bytes)
        self.reset_enabled = False
        self.reset_count = 0
        self.read_count = 0
        self.last_mode_byte = 0
        self._reset_fsm()

    def _reset_fsm(self):
        self.phase = "CMD"
        self.sck_cnt = 0
        self.cmd_reg = 0
        self.addr_reg = 0
        self.mode_reg = 0
        self.driving = False
        self.out_nib = 0

    def load_image(self, data: bytes, offset: int = 0):
        end = offset + len(data)
        if end > len(self.mem):
            self.mem.extend(b"\xFF" * (end - len(self.mem)))
        self.mem[offset:end] = data

    def on_cs_high(self):
        if self.phase == "CMD" and self.sck_cnt == 8:
            if self.cmd_reg == 0x66:
                self.reset_enabled = True
            elif self.cmd_reg == 0x99 and self.reset_enabled:
                self.reset_count += 1
                self.reset_enabled = False
        self._reset_fsm()

    def on_sck_rise(self, sd_in: int):
        """Sample host-driven SD lines on rising SCK."""
        if self.phase == "CMD":
            mosi = sd_in & 0x1
            self.cmd_reg = ((self.cmd_reg << 1) | mosi) & 0xFF
            self.sck_cnt += 1
            if self.sck_cnt == 8:
                if self.cmd_reg == 0xEB:
                    self.phase = "ADDR"
                    self.sck_cnt = 0
                    self.addr_reg = 0
                else:
                    # Stay in CMD for 66h/99h until CS rises
                    pass
        elif self.phase == "ADDR":
            self.addr_reg = ((self.addr_reg << 4) | (sd_in & 0xF)) & 0xFFFFFF
            self.sck_cnt += 1
            if self.sck_cnt == 6:
                self.phase = "MODE"
                self.sck_cnt = 0
                self.mode_reg = 0
        elif self.phase == "MODE":
            self.mode_reg = ((self.mode_reg << 4) | (sd_in & 0xF)) & 0xFF
            self.sck_cnt += 1
            if self.sck_cnt == 2:
                self.last_mode_byte = self.mode_reg
                self.phase = "DUMMY"
                self.sck_cnt = 0
        elif self.phase == "DUMMY":
            self.sck_cnt += 1

    def on_sck_fall(self):
        """Update output nibble on falling SCK during DUMMY->DATA and DATA phases."""
        if self.phase == "DUMMY" and self.sck_cnt == 4:
            self.phase = "DATA"
            self.sck_cnt = 0
            self.read_count += 1
            b_val = self.mem[self.addr_reg % len(self.mem)]
            self.out_nib = (b_val >> 4) & 0xF
            self.driving = True
        elif self.phase == "DATA":
            self.sck_cnt += 1
            b_val = self.mem[self.addr_reg % len(self.mem)]
            if (self.sck_cnt & 1) == 1:
                self.out_nib = b_val & 0xF
            else:
                self.addr_reg = (self.addr_reg + 1) & 0xFFFFFF
                b_next = self.mem[self.addr_reg % len(self.mem)]
                self.out_nib = (b_next >> 4) & 0xF
            self.driving = True
