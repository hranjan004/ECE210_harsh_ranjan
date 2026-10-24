![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# NILM-TT: A Streaming INT8 LSTM Load-Disaggregation Engine for Tiny Tapeout

**Top Module:** `tt_um_harsh_nilm`

NILM-TT is a descriptor-driven INT8 recurrent neural network (LSTM / GRU) inference coprocessor designed for non-intrusive load monitoring (NILM) on residential electrical panels:
- **Streaming INT8 Datapath:** Single time-multiplexed signed 8x8 MAC (`nilm_mac`) with 24-bit saturating accumulator, round-half-up power-of-two requantization, and multiplierless PLAN piecewise-linear (PWL) `sigmoid` and `tanh` activations (`nilm_pwl`, `nilm_cell`).
- **Unified 1-4-4 QSPI Memory Controller (`nilm_qspi`):** Streams panel descriptors, weights, biases, and thresholds from the Tiny Tapeout QSPI Pmod Flash (`CS0_FLASH_N`, `EBh` quad read) and saves/restores 100-byte per-panel recurrent contexts ($h, c$, debounce history) in QSPI PSRAM A (`CS1_RAM_A_N`, `EBh`/`38h` with automatic `PSRAM_BURST_MAX` chunking for $t_{\text{CEM}} \le 8\ \mu\text{s}$ and `BUS_REQ`/`BUS_GNT` handover).
- **Oversampled Host SPI Slave (`nilm_host_spi`, `nilm_cmd`, `nilm_csr`):** Single-clock-domain Mode-0 SPI slave supporting `LOAD_X`, `RUN`, `RUN_ALL`, `RD_RESULT`, `RD_LOGIT`, `QSPI_INIT`, `STEP_UNIT`, optional CRC-8 frame validation, and full CSR buffer windows (`U_WIN`, `HB_WIN`, `C_WIN`).
- **Bit-Exact Verification:** Validated against the integer NumPy golden model (`golden/nilm_golden.py`) across unit, protocol, cell, multi-panel LSTM/GRU, debounce, and gate-level smoke cocotb test suites (`test/test.py`).

## Repository Structure

- `src/`: Synthesizable Verilog-2005 RTL (`project.v`, `nilm_pkg.vh`, `nilm_host_spi.v`, `nilm_cmd.v`, `nilm_csr.v`, `nilm_qspi.v`, `nilm_seq.v`, `nilm_mac.v`, `nilm_pwl.v`, `nilm_cell.v`, `nilm_head.v`, `nilm_vram.v`)
- `golden/`: Bit-exact Python integer golden model (`fixedpoint.py`, `pwl.py`, `blob.py`, `nilm_golden.py`) and unit tests (`test_golden.py`)
- `test/`: Cocotb testbench (`tb.v`, `test.py`) and pin-accurate behavioural models (`models/qspi_flash.py`, `models/psram_aps6404.py`, `models/host_spi_driver.py`)
- `docs/info.md`: Tiny Tapeout project datasheet

## Running Simulations Locally

```bash
# Run Verilator + Icarus Verilog lint, Python golden unit tests, and cocotb RTL tests
make lint
make golden
make test
make synth
```
