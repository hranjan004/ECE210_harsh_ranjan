/*
 * Host SPI Command Decoder (Sec. 7.2, 8.1)
 */
`default_nettype none

`ifndef NILM_PKG_VH
`define NILM_PKG_VH
`define NILM_ID_BYTE0       8'h4E
`define NILM_ID_BYTE1       8'h54
`define NILM_VERSION        8'h10
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
`define ERR_NONE            8'h00
`define ERR_FRAME           8'h01
`define ERR_OPCODE          8'h02
`define ERR_MAGIC           8'h10
`define ERR_DESC            8'h11
`define ERR_BUS_BUSY        8'h20
`define ERR_ABORT           8'h30
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

module nilm_cmd (
    input  wire               clk,
    input  wire               rst_n,

    // From nilm_host_spi
    input  wire               frame_start,
    input  wire               frame_end,
    input  wire               frame_err,
    input  wire               rx_valid,
    input  wire        [7:0]  rx_byte,
    output reg         [7:0]  tx_byte,

    // Configuration
    input  wire               crc_en,
    input  wire               seq_busy,

    // CSR interface
    output reg                csr_we,
    output reg                csr_re,
    output reg         [7:0]  csr_addr,
    output reg         [7:0]  csr_wdata,
    input  wire        [7:0]  csr_rdata,

    // Error & bus handover pulses to CSR
    output reg                cmd_err_pulse,
    output reg         [7:0]  cmd_err_code,
    output reg                cmd_bus_rel,
    output reg                cmd_bus_acq,

    // Per-panel results & logits for RD_RESULT / RD_LOGIT
    input  wire        [15:0] panel_res_0,
    input  wire        [15:0] panel_res_1,
    input  wire        [15:0] panel_res_2,
    input  wire        [15:0] panel_res_3,
    output reg         [3:0]  logit_raddr,
    input  wire signed [15:0] logit_rdata,

    // Host access to vram (LOAD_X, LOAD_CTX, READ_CTX)
    output reg                host_ctx_clear,
    output reg                host_ctx_loaded,
    input  wire               seq_done_pulse,

    output reg                host_u_we,
    output reg         [5:0]  host_u_waddr,
    output reg  signed [7:0]  host_u_wdata,
    output reg         [5:0]  host_u_raddr,
    input  wire signed [7:0]  host_u_rdata,

    output reg                host_hb_we,
    output reg         [4:0]  host_hb_waddr,
    output reg  signed [7:0]  host_hb_wdata,
    output reg         [4:0]  host_hb_raddr,
    input  wire signed [7:0]  host_hb_rdata,

    output reg                host_c_we,
    output reg         [4:0]  host_c_waddr,
    output reg  signed [15:0] host_c_wdata,
    output reg         [4:0]  host_c_raddr,
    input  wire signed [15:0] host_c_rdata,

    output reg                host_deb_we,
    output reg         [3:0]  host_deb_waddr,
    output reg         [3:0]  host_deb_wdata,
    output reg         [1:0]  host_deb_pair_idx,
    input  wire        [7:0]  host_deb_pair_rdata,

    // Sequencer command triggers
    output reg                cmd_init_trig,
    output reg                cmd_run_trig,
    output reg                cmd_run_all_trig,
    output reg                cmd_step_trig,
    output reg         [1:0]  cmd_panel_arg,
    output reg                cmd_mem_rd_trig,
    output reg                cmd_mem_wr_trig,
    output reg                cmd_mem_dev,
    output reg         [23:0] cmd_mem_addr,
    output reg         [7:0]  cmd_mem_len
);

  reg [7:0] opcode;
  reg [7:0] byte_idx;
  reg [7:0] arg0;
  reg [7:0] arg1;
  reg [7:0] arg2;
  reg [7:0] arg3;
  reg [7:0] arg4;
  reg [7:0] c_lo_buf;

  reg [7:0] crc_reg;
  function automatic [7:0] next_crc8;
    input [7:0] c;
    input [7:0] d;
    reg [7:0] v;
    integer b;
    begin
      v = c ^ d;
      for (b = 0; b < 8; b = b + 1) begin
        if (v[7]) v = {v[6:0], 1'b0} ^ 8'h07;
        else      v = {v[6:0], 1'b0};
      end
      next_crc8 = v;
    end
  endfunction

  reg       pend_wr_reg;
  reg [7:0] pend_wr_addr;
  reg [7:0] pend_wr_data;
  reg       pend_run;
  reg       pend_run_all;
  reg       pend_init;
  reg       pend_step;
  reg       pend_ctx_rst;
  reg       is_write_op;
  reg       frame_crc_en;
  reg       host_deb_we_hi;
  reg [3:0] host_deb_base;

  reg [15:0] r_res;
  always @(*) begin
    case (arg0[1:0])
      2'd0: r_res = panel_res_0;
      2'd1: r_res = panel_res_1;
      2'd2: r_res = panel_res_2;
      default: r_res = panel_res_3;
    endcase
  end

  wire [5:0] ctx_h_off   = byte_idx[5:0] - 6'd2;
  wire [5:0] ctx_c_sub   = byte_idx[5:0] - 6'd34;
  wire [4:0] ctx_c_off   = ctx_c_sub[5:1];
  wire [2:0] ctx_deb_off = byte_idx[2:0] - 3'd2; // 98 & 7 == 2

  always @(*) begin
    csr_addr          = arg0;
    logit_raddr       = arg1[3:0];
    host_u_raddr      = 6'd0;
    host_hb_raddr     = 5'd0;
    host_c_raddr      = 5'd0;
    host_deb_pair_idx = 2'd0;

    if (rx_valid && (byte_idx == 8'd1) && (opcode == `OP_RD_REG || opcode == `OP_RD_REG_BURST)) begin
      csr_addr = rx_byte;
    end else if (opcode == `OP_RD_REG_BURST && byte_idx >= 8'd3) begin
      csr_addr = arg0 + (byte_idx - 8'd3);
    end else if (csr_we) begin
      csr_addr = pend_wr_addr;
    end

    if (opcode == `OP_RD_LOGIT && byte_idx >= 8'd2) begin
      logit_raddr = (rx_valid && byte_idx == 8'd2) ? rx_byte[3:0] : arg1[3:0];
    end

    if (opcode == `OP_READ_CTX) begin
      if (byte_idx >= 8'd2 && byte_idx < 8'd34) begin
        host_hb_raddr = ctx_h_off[4:0];
        host_u_raddr  = 6'd32 + ctx_h_off;
      end else if (byte_idx >= 8'd34 && byte_idx < 8'd98) begin
        host_c_raddr = ctx_c_off;
      end else if (byte_idx >= 8'd98 && byte_idx < 8'd102) begin
        host_deb_pair_idx = ctx_deb_off[1:0];
      end
    end
  end

  wire [7:0] load_x_idx = byte_idx - 8'd3;
  wire [7:0] mem_wr_idx = byte_idx - 8'd6;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      opcode           <= 8'h00;
      byte_idx         <= 8'd0;
      arg0             <= 8'h00;
      arg1             <= 8'h00;
      arg2             <= 8'h00;
      arg3             <= 8'h00;
      arg4             <= 8'h00;
      c_lo_buf         <= 8'h00;
      tx_byte          <= 8'h00;
      crc_reg          <= 8'h00;
      csr_we           <= 1'b0;
      csr_re           <= 1'b0;
      csr_wdata        <= 8'h00;
      cmd_err_pulse    <= 1'b0;
      cmd_err_code     <= `ERR_NONE;
      cmd_bus_rel      <= 1'b0;
      cmd_bus_acq      <= 1'b0;
      host_ctx_clear   <= 1'b0;
      host_ctx_loaded  <= 1'b0;
      host_u_we        <= 1'b0;
      host_u_waddr     <= 6'd0;
      host_u_wdata     <= 8'sd0;
      host_hb_we       <= 1'b0;
      host_hb_waddr    <= 5'd0;
      host_hb_wdata    <= 8'sd0;
      host_c_we        <= 1'b0;
      host_c_waddr     <= 5'd0;
      host_c_wdata     <= 16'sd0;
      host_deb_we      <= 1'b0;
      host_deb_we_hi   <= 1'b0;
      host_deb_base    <= 4'd0;
      host_deb_waddr   <= 4'd0;
      host_deb_wdata   <= 4'd0;
      cmd_init_trig    <= 1'b0;
      cmd_run_trig     <= 1'b0;
      cmd_run_all_trig <= 1'b0;
      cmd_step_trig    <= 1'b0;
      cmd_panel_arg    <= 2'd0;
      cmd_mem_rd_trig  <= 1'b0;
      cmd_mem_wr_trig  <= 1'b0;
      cmd_mem_dev      <= 1'b0;
      cmd_mem_addr     <= 24'd0;
      cmd_mem_len      <= 8'd0;
      pend_wr_reg      <= 1'b0;
      pend_wr_addr     <= 8'h00;
      pend_wr_data     <= 8'h00;
      pend_run         <= 1'b0;
      pend_run_all     <= 1'b0;
      pend_init        <= 1'b0;
      pend_step        <= 1'b0;
      pend_ctx_rst     <= 1'b0;
      is_write_op      <= 1'b0;
      frame_crc_en     <= 1'b0;
    end else begin
      csr_we           <= 1'b0;
      csr_re           <= 1'b0;
      cmd_err_pulse    <= 1'b0;
      cmd_bus_rel      <= 1'b0;
      cmd_bus_acq      <= 1'b0;
      host_ctx_clear   <= 1'b0;
      host_u_we        <= 1'b0;
      host_hb_we       <= 1'b0;
      host_c_we        <= 1'b0;
      host_deb_we      <= 1'b0;
      host_deb_we_hi   <= 1'b0;
      cmd_init_trig    <= 1'b0;
      cmd_run_trig     <= 1'b0;
      cmd_run_all_trig <= 1'b0;
      cmd_step_trig    <= 1'b0;
      cmd_mem_rd_trig  <= 1'b0;
      cmd_mem_wr_trig  <= 1'b0;

      if (seq_done_pulse) begin
        host_ctx_loaded <= 1'b0;
      end

      if (host_deb_we && !host_deb_we_hi) begin
        host_deb_we    <= 1'b1;
        host_deb_we_hi <= 1'b1;
        host_deb_waddr <= host_deb_base + 4'd1;
        host_deb_wdata <= arg4[7:4];
      end

      if (frame_err) begin
        cmd_err_pulse <= 1'b1;
        cmd_err_code  <= `ERR_FRAME;
        pend_wr_reg   <= 1'b0;
        pend_run      <= 1'b0;
        pend_run_all  <= 1'b0;
        pend_init     <= 1'b0;
        pend_step     <= 1'b0;
        pend_ctx_rst  <= 1'b0;
      end else if (frame_start) begin
        byte_idx     <= 8'd0;
        tx_byte      <= 8'h00;
        crc_reg      <= 8'h00;
        pend_wr_reg  <= 1'b0;
        pend_run     <= 1'b0;
        pend_run_all <= 1'b0;
        pend_init    <= 1'b0;
        pend_step    <= 1'b0;
        pend_ctx_rst <= 1'b0;
        is_write_op  <= 1'b0;
        frame_crc_en <= crc_en;
      end else if (frame_end) begin
        if (frame_crc_en && is_write_op) begin
          if (crc_reg != 8'h00) begin
            cmd_err_pulse <= 1'b1;
            cmd_err_code  <= `ERR_FRAME;
          end else begin
            if (pend_wr_reg) begin
              csr_we    <= 1'b1;
              csr_wdata <= pend_wr_data;
            end
            if (pend_run)     cmd_run_trig     <= 1'b1;
            if (pend_run_all) cmd_run_all_trig <= 1'b1;
            if (pend_init)    cmd_init_trig    <= 1'b1;
            if (pend_step)    cmd_step_trig    <= 1'b1;
            if (pend_ctx_rst) host_ctx_clear   <= 1'b1;
          end
        end
        pend_wr_reg  <= 1'b0;
        pend_run     <= 1'b0;
        pend_run_all <= 1'b0;
        pend_init    <= 1'b0;
        pend_step    <= 1'b0;
        pend_ctx_rst <= 1'b0;
      end else if (rx_valid) begin
        crc_reg  <= next_crc8(crc_reg, rx_byte);
        byte_idx <= byte_idx + 8'd1;

        if (byte_idx == 8'd0) begin
          opcode <= rx_byte;
          case (rx_byte)
            `OP_ID: begin
              tx_byte     <= 8'h00;
              is_write_op <= 1'b0;
            end
            `OP_WR_REG, `OP_LOAD_X, `OP_LOAD_CTX,
            `OP_RUN, `OP_RUN_ALL, `OP_CTX_RESET,
            `OP_QSPI_INIT, `OP_BUS_RELEASE, `OP_BUS_ACQUIRE,
            `OP_MEM_WR, `OP_STEP_UNIT: begin
              is_write_op <= 1'b1;
              if (rx_byte == `OP_RUN_ALL) begin
                if (frame_crc_en) pend_run_all <= 1'b1;
                else              cmd_run_all_trig <= 1'b1;
              end else if (rx_byte == `OP_QSPI_INIT) begin
                cmd_bus_acq <= 1'b1;
                if (frame_crc_en) pend_init <= 1'b1;
                else              cmd_init_trig <= 1'b1;
              end else if (rx_byte == `OP_BUS_RELEASE) begin
                if (seq_busy) begin
                  cmd_err_pulse <= 1'b1;
                  cmd_err_code  <= `ERR_BUS_BUSY;
                end else begin
                  cmd_bus_rel <= 1'b1;
                end
              end else if (rx_byte == `OP_BUS_ACQUIRE) begin
                cmd_bus_acq <= 1'b1;
              end else if (rx_byte == `OP_STEP_UNIT) begin
                if (frame_crc_en) pend_step <= 1'b1;
                else              cmd_step_trig <= 1'b1;
              end
            end
            `OP_RD_REG, `OP_RD_REG_BURST, `OP_READ_CTX,
            `OP_RD_RESULT, `OP_RD_LOGIT, `OP_MEM_RD: begin
              is_write_op <= 1'b0;
            end
            default: begin
              cmd_err_pulse <= 1'b1;
              cmd_err_code  <= `ERR_OPCODE;
            end
          endcase
        end else begin
          case (opcode)
            `OP_ID: begin
              if (byte_idx == 8'd1)      tx_byte <= `NILM_ID_BYTE0;
              else if (byte_idx == 8'd2) tx_byte <= `NILM_ID_BYTE1;
              else if (byte_idx == 8'd3) tx_byte <= `NILM_VERSION;
              else                       tx_byte <= 8'h00;
            end

            `OP_WR_REG: begin
              if (byte_idx == 8'd1) begin
                arg0         <= rx_byte;
                pend_wr_addr <= rx_byte;
              end else if (byte_idx == 8'd2) begin
                pend_wr_data <= rx_byte;
                if (frame_crc_en) begin
                  pend_wr_reg <= 1'b1;
                end else begin
                  csr_we    <= 1'b1;
                  csr_wdata <= rx_byte;
                end
              end
            end

            `OP_RD_REG: begin
              if (byte_idx == 8'd1) begin
                arg0    <= rx_byte;
                tx_byte <= 8'h00;
              end else if (byte_idx == 8'd2) begin
                tx_byte <= csr_rdata;
                csr_re  <= 1'b1;
              end
            end

            `OP_RD_REG_BURST: begin
              if (byte_idx == 8'd1) begin
                arg0 <= rx_byte;
              end else if (byte_idx == 8'd2) begin
                arg1    <= rx_byte;
                tx_byte <= 8'h00;
              end else if ((byte_idx - 8'd3) < arg1) begin
                tx_byte <= csr_rdata;
                csr_re  <= 1'b1;
              end
            end

            `OP_LOAD_X: begin
              if (byte_idx == 8'd1) begin
                arg0          <= rx_byte;
                cmd_panel_arg <= rx_byte[1:0];
              end else if (byte_idx == 8'd2) begin
                arg1 <= rx_byte;
              end else if (load_x_idx < arg1) begin
                host_u_we    <= 1'b1;
                host_u_waddr <= load_x_idx[5:0];
                host_u_wdata <= $signed(rx_byte);
              end
            end

            `OP_LOAD_CTX: begin
              if (byte_idx == 8'd1) begin
                arg0            <= rx_byte;
                cmd_panel_arg   <= rx_byte[1:0];
                host_ctx_loaded <= 1'b1;
              end else if (byte_idx >= 8'd2 && byte_idx < 8'd34) begin
                host_hb_we    <= 1'b1;
                host_hb_waddr <= ctx_h_off[4:0];
                host_hb_wdata <= $signed(rx_byte);
                host_u_we     <= 1'b1;
                host_u_waddr  <= 6'd32 + ctx_h_off;
                host_u_wdata  <= $signed(rx_byte);
              end else if (byte_idx >= 8'd34 && byte_idx < 8'd98) begin
                if (byte_idx[0] == 1'b0) begin
                  c_lo_buf <= rx_byte;
                end else begin
                  host_c_we    <= 1'b1;
                  host_c_waddr <= ctx_c_off;
                  host_c_wdata <= $signed({rx_byte, c_lo_buf});
                end
              end else if (byte_idx >= 8'd98 && byte_idx < 8'd102) begin
                arg4           <= rx_byte;
                host_deb_base  <= {ctx_deb_off, 1'b0};
                host_deb_we    <= 1'b1;
                host_deb_we_hi <= 1'b0;
                host_deb_waddr <= {ctx_deb_off, 1'b0};
                host_deb_wdata <= rx_byte[3:0];
              end
            end

            `OP_READ_CTX: begin
              if (byte_idx == 8'd1) begin
                arg0    <= rx_byte;
                tx_byte <= 8'h00;
              end else if (byte_idx >= 8'd2 && byte_idx < 8'd34) begin
                tx_byte <= host_ctx_loaded ? host_hb_rdata : host_u_rdata;
              end else if (byte_idx >= 8'd34 && byte_idx < 8'd98) begin
                tx_byte <= (byte_idx[0] == 1'b0) ? host_c_rdata[7:0] : host_c_rdata[15:8];
              end else if (byte_idx >= 8'd98 && byte_idx < 8'd102) begin
                tx_byte <= host_deb_pair_rdata;
              end
            end

            `OP_RUN: begin
              if (byte_idx == 8'd1) begin
                cmd_panel_arg <= rx_byte[1:0];
                if (frame_crc_en) pend_run <= 1'b1;
                else              cmd_run_trig <= 1'b1;
              end
            end

            `OP_RD_RESULT: begin
              if (byte_idx == 8'd1) begin
                arg0    <= rx_byte;
                tx_byte <= 8'h00;
              end else if (byte_idx == 8'd2) begin
                tx_byte <= r_res[7:0];
              end else if (byte_idx == 8'd3) begin
                tx_byte <= r_res[15:8];
              end
            end

            `OP_RD_LOGIT: begin
              if (byte_idx == 8'd1) begin
                arg0 <= rx_byte;
              end else if (byte_idx == 8'd2) begin
                arg1    <= rx_byte;
                tx_byte <= 8'h00;
              end else if (byte_idx == 8'd3) begin
                tx_byte <= logit_rdata[7:0];
              end else if (byte_idx == 8'd4) begin
                tx_byte <= logit_rdata[15:8];
              end
            end

            `OP_CTX_RESET: begin
              if (byte_idx == 8'd1) begin
                cmd_panel_arg   <= rx_byte[1:0];
                host_ctx_loaded <= 1'b0;
                if (frame_crc_en) pend_ctx_rst <= 1'b1;
                else              host_ctx_clear <= 1'b1;
              end
            end

            `OP_MEM_RD, `OP_MEM_WR: begin
              if (byte_idx == 8'd1)      arg0 <= rx_byte;
              else if (byte_idx == 8'd2) arg1 <= rx_byte;
              else if (byte_idx == 8'd3) arg2 <= rx_byte;
              else if (byte_idx == 8'd4) arg3 <= rx_byte;
              else if (byte_idx == 8'd5) begin
                arg4         <= rx_byte;
                cmd_mem_dev  <= arg0[0];
                cmd_mem_addr <= {arg1, arg2, arg3};
                cmd_mem_len  <= rx_byte;
                if (opcode == `OP_MEM_RD) begin
                  cmd_mem_rd_trig <= 1'b1;
                end
              end else if (opcode == `OP_MEM_WR && mem_wr_idx < arg4) begin
                host_u_we    <= 1'b1;
                host_u_waddr <= mem_wr_idx[5:0];
                host_u_wdata <= $signed(rx_byte);
                if ((byte_idx - 8'd5) == arg4) begin
                  cmd_mem_wr_trig <= 1'b1;
                end
              end
            end

            default: ;
          endcase
        end
      end
    end
  end

  wire _unused = &{ctx_c_sub[0], 1'b0};

endmodule
