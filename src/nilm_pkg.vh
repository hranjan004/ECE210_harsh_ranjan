/*
 * NILM-TT Package Definitions (Opcodes, CSR Addresses, Sequencer States, Errors)
 */
`ifndef NILM_PKG_VH
`define NILM_PKG_VH

// Chip ID and Version (§7.2, §8.8)
`define NILM_ID_BYTE0       8'h4E   // 'N'
`define NILM_ID_BYTE1       8'h54   // 'T'
`define NILM_VERSION        8'h10   // v1.0

// Host SPI Opcodes (§7.2)
`define OP_WR_REG           8'h01
`define OP_RD_REG           8'h02
`define OP_RD_REG_BURST     8'h03
`define OP_LOAD_X           8'h10
`define OP_LOAD_CTX         8'h11
`define OP_READ_CTX         8'h12
`define OP_RUN              8'h20
`define OP_RUN_ALL          8'h21
`define OP_RD_RESULT        8'h30
`define OP_RD_LOGIT         8'h31
`define OP_CTX_RESET        8'h40
`define OP_QSPI_INIT        8'h50
`define OP_BUS_RELEASE      8'h60
`define OP_BUS_ACQUIRE      8'h61
`define OP_MEM_RD           8'h70
`define OP_MEM_WR           8'h71
`define OP_STEP_UNIT        8'h80
`define OP_ID               8'hA5

// CSR Map (§8.8)
`define CSR_ID              8'h00
`define CSR_VERSION         8'h01
`define CSR_STATUS          8'h02
`define CSR_CTRL            8'h03
`define CSR_PANEL_SEL       8'h04
`define CSR_QSPI_CFG        8'h05
`define CSR_FLASH_BASE_L    8'h06
`define CSR_FLASH_BASE_M    8'h07
`define CSR_FLASH_BASE_H    8'h08
`define CSR_CTX_BASE_L      8'h09
`define CSR_CTX_BASE_M      8'h0A
`define CSR_CTX_BASE_H      8'h0B
`define CSR_PSRAM_BURST_MAX 8'h0C
`define CSR_INIT_WAIT       8'h0D
`define CSR_ERR_CODE        8'h0E
`define CSR_RESULT_LO       8'h10
`define CSR_RESULT_HI       8'h11
`define CSR_NHEAD           8'h12
`define CSR_CYCLES_L        8'h14
`define CSR_CYCLES_M        8'h15
`define CSR_CYCLES_H        8'h16
`define CSR_DBG_STATE       8'h18
`define CSR_DBG_UNIT        8'h19
`define CSR_DBG_ACC_L       8'h1A
`define CSR_DBG_ACC_M       8'h1B
`define CSR_DBG_ACC_H       8'h1C

// Error Codes (§8.8)
`define ERR_NONE            8'h00
`define ERR_FRAME           8'h01
`define ERR_OPCODE          8'h02
`define ERR_MAGIC           8'h10
`define ERR_DESC            8'h11
`define ERR_BUS_BUSY        8'h20
`define ERR_ABORT           8'h30

// Sequencer FSM States (27 states, 5-bit, §8.6)
`define ST_IDLE             5'd0
`define ST_INIT_RST         5'd1
`define ST_INIT_WAIT        5'd2
`define ST_CHK_MAGIC        5'd3
`define ST_DESC_FETCH       5'd4
`define ST_CTX_LOAD         5'd5
`define ST_GATE_BIAS        5'd6
`define ST_GATE_MAC         5'd7
`define ST_GATE_ACT         5'd8
`define ST_CELL_FC          5'd9
`define ST_CELL_IG          5'd10
`define ST_CELL_SUM         5'd11
`define ST_CELL_TANH        5'd12
`define ST_CELL_H           5'd13
`define ST_UNIT_NEXT        5'd14
`define ST_H_COMMIT         5'd15
`define ST_DENSE_BIAS       5'd16
`define ST_DENSE_MAC        5'd17
`define ST_DENSE_ACT        5'd18
`define ST_HEAD_BIAS        5'd19
`define ST_HEAD_MAC         5'd20
`define ST_HEAD_CMP         5'd21
`define ST_DEBOUNCE         5'd22
`define ST_CTX_SAVE         5'd23
`define ST_DONE             5'd24
`define ST_ERR              5'd25
`define ST_BUS_HOST         5'd26

`endif
