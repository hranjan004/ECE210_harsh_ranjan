<!---
This file is used to generate your project datasheet.
-->

## How it works

**NILM-TT (`tt_um_harsh_nilm`)** is a descriptor-driven INT8 recurrent neural network (LSTM / GRU) inference coprocessor designed for real-time Non-Intrusive Load Monitoring (NILM) on residential electrical panels.

Every 6 seconds, an external microcontroller (such as the RP2350 on the Tiny Tapeout demo board) sends a quantized INT8 feature vector ($F \le 32$, default $F = 14$) over an oversampled Mode-0 Host SPI-slave interface (`ui_in[2:0]`, `uo_out[0]`). When triggered via the `RUN` / `RUN_ALL` SPI command or `RUN_PIN` (`ui_in[4]`), the 27-state microsequencer (`nilm_seq`):

1. **Fetches the Panel Descriptor** (64 bytes, magic `"NTPD"`) from the external QSPI Flash (`CS0_FLASH_N`) using unified 1-4-4 `EBh` quad-read bursts.
2. **Loads Per-Panel Context** (100 bytes: 32-byte hidden vector $h_{t-1}$, 64-byte INT16 cell state $c_{t-1}$, and 4-byte packed debounce history) from QSPI PSRAM A (`CS1_RAM_A_N`), or uses on-chip context preloaded by the host in Host-Context mode (`CTX_MODE=1`).
3. **Executes One Recurrent Step ($H \le 16$)** through a time-multiplexed signed INT8 MAC (`nilm_mac`) with 24-bit saturating accumulation, round-half-up power-of-two requantization, and multiplierless PLAN piecewise-linear (PWL) `sigmoid` and `tanh` activations (`nilm_pwl`, `nilm_cell`). Both 4-gate LSTM (`CELL_TYPE=0`) and 3-gate reset-gated GRU (`CELL_TYPE=1`) cells are supported.
4. **Computes the Shared Dense ReLU Layer ($N_Z \le 16$)** and **Linear Logit Heads ($N_{\text{HEAD}} \le 8$)**, comparing each INT16 logit against a per-head logit-space threshold $\theta_m = \ln(p_m / (1 - p_m))$ and applying an optional $N$-of-$M$ ($M \le 4$) popcount debounce filter (`nilm_head`).
5. **Saves Updated Context** back to QSPI PSRAM A in `38h` quad-write bursts automatically chunked to `PSRAM_BURST_MAX` bytes with a host-selected burst limit. The host must choose this limit for the actual chip clock and SCK divider to meet $t_{\text{CEM}} = 8\ \mu\text{s}$; chunking alone does not guarantee it.

## How to test

1. **Initialize QSPI Memory:** Hold `BUS_REQ=1` (or Reset) while programming the `"NILM"` model blob into QSPI Flash using the RP2350 programmer, then release `BUS_REQ=0` and issue opcode `0x50` (`QSPI_INIT`) over Host SPI (`HOST_CS_N`, `HOST_SCK`, `HOST_MOSI`, `HOST_MISO`). Verify `STATUS.QSPI_READY` (`0x02[4]`) asserts without error.
2. **Load Features & Run Inference:** Send opcode `0x10` (`LOAD_X`) with panel index $p \in \{0, 1, 2\}$ and $F$ signed INT8 feature bytes, followed by opcode `0x20` (`RUN`, panel $p$) or `0x21` (`RUN_ALL`). Wait for `BUSY` (`uo_out[2]`) to fall or `IRQ` (`uo_out[1]`) to rise.
3. **Read Results & Logits:** Issue opcode `0x30` (`RD_RESULT`, panel $p$) to read the 16-bit appliance ON/OFF bitmask, and opcode `0x31` (`RD_LOGIT`, panel $p$, head $m$) to inspect exact INT16 pre-threshold logits.
4. **Run Simulation Locally:**
   - Python bit-exact golden model tests: `pytest golden/`
   - RTL cocotb verification suites: `cd test && make -B`

## External hardware

- **Tiny Tapeout QSPI Pmod** connected to `uio[7:0]`:
  - 1x Winbond `W25Q128JV` (16 MB QSPI Flash on `CS0` / `uio[0]`) storing model descriptors, weights, biases, and thresholds.
  - 2x AP Memory `APS6404L-3SQR` (2x 8 MB QSPI PSRAM on `CS1` / `uio[6]` and `CS2` / `uio[7]`) storing per-panel recurrent states ($h, c$, and debounce history).
- **RP2350 (on Tiny Tapeout Demo Board)** or external MCU driving the Host SPI port on `ui_in[2:0]` and `uo_out[0]`.

## Implemented capacity and timing limits

The optimized implementation accepts F=1..32, H=1..16, NZ=1..16, and
NHEAD=1..8. Other dimensions return ERR_DESC (0x11). The blob packer checks
these limits before serializing weights; its default is H=16. The external
context remains 100 bytes for compatibility, with unused h/c entries zeroed
when saved. Eight heads is the limit because the context carries four packed
debounce bytes and RD_LOGIT stores eight logits. A 14-head panel requires a
versioned context/logit expansion; it cannot run intact on this configuration.

At a 50 MHz chip clock, tested PSRAM_BURST_MAX settings for SCK divisors
2, 4, 8, and 16 are respectively 64, 32, 8, and 1 byte. Recompute the limit
at other chip clocks. The read preamble alone costs 20 SCK cycles; include
the data cycles (two per byte) and controller tail cycles in the CE-low
budget. Setting a slower SCK without reducing the burst limit can violate
8 microseconds. Use Host-Context mode when the external-memory timing budget
cannot be met. Local simulation uses behavioral memory models, not silicon
or board timing measurements.
