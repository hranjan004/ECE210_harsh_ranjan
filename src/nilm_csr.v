/*
 * Control & Status Register (CSR) File and Buffer Window (§8.8)
 */
`default_nettype none
`include "nilm_pkg.vh"

module nilm_csr (
    input  wire               clk,
    input  wire               rst_n,

    // Read/write interface from cmd_decoder
    input  wire               csr_we,
    input  wire               csr_re,
    input  wire        [7:0]  csr_addr,
    input  wire        [7:0]  csr_wdata,
    output reg         [7:0]  csr_rdata,

    // Direct error / command pulses from cmd_decoder
    input  wire               cmd_err_pulse,
    input  wire        [7:0]  cmd_err_code,
    input  wire               cmd_bus_rel,
    input  wire               cmd_bus_acq,

    // Status inputs from Sequencer / QSPI / MAC
    input  wire               seq_busy,
    input  wire               seq_done_pulse,
    input  wire               seq_err_pulse,
    input  wire        [7:0]  seq_err_code,
    input  wire               bus_gnt,
    input  wire               qspi_ready,
    input  wire               sat_seen,
    input  wire        [23:0] last_cycles,
    input  wire        [4:0]  dbg_state,
    input  wire        [4:0]  dbg_unit,
    input  wire signed [23:0] dbg_acc,

    // Per-panel results & head counts from Sequencer
    input  wire        [15:0] panel_res_0,
    input  wire        [15:0] panel_res_1,
    input  wire        [15:0] panel_res_2,
    input  wire        [15:0] panel_res_3,
    input  wire        [4:0]  panel_nhead_0,
    input  wire        [4:0]  panel_nhead_1,
    input  wire        [4:0]  panel_nhead_2,
    input  wire        [4:0]  panel_nhead_3,

    // Buffer window read/write ports into vram (U_WIN 0x40-0x7F, HB_WIN 0x80-0x9F, C_WIN 0xA0-0xDF)
    output wire               win_u_we,
    output wire        [5:0]  win_u_addr,
    output wire signed [7:0]  win_u_wdata,
    input  wire signed [7:0]  win_u_rdata,

    output wire               win_hb_we,
    output wire        [4:0]  win_hb_addr,
    output wire signed [7:0]  win_hb_wdata,
    input  wire signed [7:0]  win_hb_rdata,

    output wire               win_c_we,
    output wire        [4:0]  win_c_addr,
    output wire signed [15:0] win_c_wdata,
    input  wire signed [15:0] win_c_rdata,

    // Configuration & control outputs
    output reg                done_sticky,
    output reg                err_sticky,
    output reg         [7:0]  err_code,
    output reg                ctrl_start_pulse,
    output reg                ctrl_abort_pulse,
    output reg                ctrl_soft_rst_pulse,
    output reg                irq_en,
    output reg                deb_en,
    output reg                crc_en,
    output reg                ctx_mode,
    output reg                step_mode,
    output reg         [1:0]  panel_sel,
    output reg         [1:0]  rd_lat,
    output reg                rd_half,
    output reg         [1:0]  sck_div,
    output reg                psram_en,
    output reg         [23:0] flash_base,
    output reg         [23:0] ctx_base,
    output reg         [7:0]  psram_burst_max,
    output reg         [7:0]  init_wait,
    output reg                host_bus_req
);

  // Selected panel result & head count
  reg [15:0] sel_res;
  reg [4:0]  sel_nhead;
  always @(*) begin
    case (panel_sel)
      2'd0: begin sel_res = panel_res_0; sel_nhead = panel_nhead_0; end
      2'd1: begin sel_res = panel_res_1; sel_nhead = panel_nhead_1; end
      2'd2: begin sel_res = panel_res_2; sel_nhead = panel_nhead_2; end
      default: begin sel_res = panel_res_3; sel_nhead = panel_nhead_3; end
    endcase
  end

  // Buffer window decoding
  assign win_u_we    = csr_we && (csr_addr >= 8'h40 && csr_addr <= 8'h7F);
  assign win_u_addr  = csr_addr[5:0];
  assign win_u_wdata = $signed(csr_wdata);

  assign win_hb_we    = csr_we && (csr_addr >= 8'h80 && csr_addr <= 8'h9F);
  assign win_hb_addr  = csr_addr[4:0];
  assign win_hb_wdata = $signed(csr_wdata);

  reg [7:0] c_lo_staging;
  assign win_c_we    = csr_we && (csr_addr >= 8'hA0 && csr_addr <= 8'hDF) && (csr_addr[0] == 1'b1);
  assign win_c_addr  = csr_addr[5:1] - 5'd16; // 0xA0>>1 = 0x50 (80); 80 - 64 = 16 -> addr[4:0] = csr_addr[5:1] ^ 5'h10
  wire [4:0] c_idx   = csr_addr[5:1] ^ 5'h10;
  assign win_c_wdata = $signed({csr_wdata, c_lo_staging});

  wire [7:0] status_byte = {
    2'b00,
    sat_seen,
    qspi_ready,
    bus_gnt,
    err_sticky,
    done_sticky,
    seq_busy
  };

  wire [7:0] ctrl_byte = {
    step_mode,
    ctx_mode,
    crc_en,
    deb_en,
    irq_en,
    3'b000
  };

  wire [7:0] qspi_cfg_byte = {
    2'b00,
    psram_en,
    sck_div,
    rd_half,
    rd_lat
  };

  // Combinational read mux
  always @(*) begin
    if (csr_addr >= 8'h40 && csr_addr <= 8'h7F) begin
      csr_rdata = win_u_rdata;
    end else if (csr_addr >= 8'h80 && csr_addr <= 8'h9F) begin
      csr_rdata = win_hb_rdata;
    end else if (csr_addr >= 8'hA0 && csr_addr <= 8'hDF) begin
      csr_rdata = csr_addr[0] ? win_c_rdata[15:8] : win_c_rdata[7:0];
    end else begin
      case (csr_addr)
        `CSR_ID:              csr_rdata = `NILM_ID_BYTE0;
        `CSR_VERSION:         csr_rdata = `NILM_VERSION;
        `CSR_STATUS:          csr_rdata = status_byte;
        `CSR_CTRL:            csr_rdata = ctrl_byte;
        `CSR_PANEL_SEL:       csr_rdata = {6'd0, panel_sel};
        `CSR_QSPI_CFG:        csr_rdata = qspi_cfg_byte;
        `CSR_FLASH_BASE_L:    csr_rdata = flash_base[7:0];
        `CSR_FLASH_BASE_M:    csr_rdata = flash_base[15:8];
        `CSR_FLASH_BASE_H:    csr_rdata = flash_base[23:16];
        `CSR_CTX_BASE_L:      csr_rdata = ctx_base[7:0];
        `CSR_CTX_BASE_M:      csr_rdata = ctx_base[15:8];
        `CSR_CTX_BASE_H:      csr_rdata = ctx_base[23:16];
        `CSR_PSRAM_BURST_MAX: csr_rdata = psram_burst_max;
        `CSR_INIT_WAIT:       csr_rdata = init_wait;
        `CSR_ERR_CODE:        csr_rdata = err_code;
        `CSR_RESULT_LO:       csr_rdata = sel_res[7:0];
        `CSR_RESULT_HI:       csr_rdata = sel_res[15:8];
        `CSR_NHEAD:           csr_rdata = {3'd0, sel_nhead};
        `CSR_CYCLES_L:        csr_rdata = last_cycles[7:0];
        `CSR_CYCLES_M:        csr_rdata = last_cycles[15:8];
        `CSR_CYCLES_H:        csr_rdata = last_cycles[23:16];
        `CSR_DBG_STATE:       csr_rdata = {3'd0, dbg_state};
        `CSR_DBG_UNIT:        csr_rdata = {3'd0, dbg_unit};
        `CSR_DBG_ACC_L:       csr_rdata = dbg_acc[7:0];
        `CSR_DBG_ACC_M:       csr_rdata = dbg_acc[15:8];
        `CSR_DBG_ACC_H:       csr_rdata = dbg_acc[23:16];
        default:              csr_rdata = 8'h00;
      endcase
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done_sticky         <= 1'b0;
      err_sticky          <= 1'b0;
      err_code            <= `ERR_NONE;
      ctrl_start_pulse    <= 1'b0;
      ctrl_abort_pulse    <= 1'b0;
      ctrl_soft_rst_pulse <= 1'b0;
      irq_en              <= 1'b0;
      deb_en              <= 1'b1;
      crc_en              <= 1'b0;
      ctx_mode            <= 1'b0;
      step_mode           <= 1'b0;
      panel_sel           <= 2'd0;
      rd_lat              <= 2'd0;
      rd_half             <= 1'b0;
      sck_div             <= 2'd0;
      psram_en            <= 1'b1;
      flash_base          <= 24'h000000;
      ctx_base            <= 24'h000000;
      psram_burst_max     <= 8'd64;
      init_wait           <= 8'd1;   // 1 * 256 clk default (fast sim, configurable up to 32)
      host_bus_req        <= 1'b0;
      c_lo_staging        <= 8'h00;
    end else begin
      ctrl_start_pulse    <= 1'b0;
      ctrl_abort_pulse    <= 1'b0;
      ctrl_soft_rst_pulse <= 1'b0;

      // Clear-on-read for DONE bit when reading CSR_STATUS
      if (csr_re && (csr_addr == `CSR_STATUS)) begin
        done_sticky <= 1'b0;
      end

      if (seq_done_pulse) begin
        done_sticky <= 1'b1;
      end

      if (cmd_err_pulse) begin
        err_sticky <= 1'b1;
        err_code   <= cmd_err_code;
      end else if (seq_err_pulse) begin
        err_sticky <= 1'b1;
        err_code   <= seq_err_code;
      end

      if (cmd_bus_rel) begin
        host_bus_req <= 1'b1;
      end else if (cmd_bus_acq) begin
        host_bus_req <= 1'b0;
      end

      if (csr_we) begin
        if (csr_addr >= 8'hA0 && csr_addr <= 8'hDF && (csr_addr[0] == 1'b0)) begin
          c_lo_staging <= csr_wdata;
        end else begin
          case (csr_addr)
            `CSR_STATUS: begin
              if (csr_wdata[1]) done_sticky <= 1'b0;
              if (csr_wdata[2]) begin
                err_sticky <= 1'b0;
                err_code   <= `ERR_NONE;
              end
            end
            `CSR_CTRL: begin
              ctrl_start_pulse    <= csr_wdata[0];
              ctrl_abort_pulse    <= csr_wdata[1];
              ctrl_soft_rst_pulse <= csr_wdata[2];
              irq_en              <= csr_wdata[3];
              deb_en              <= csr_wdata[4];
              crc_en              <= csr_wdata[5];
              ctx_mode            <= csr_wdata[6];
              step_mode           <= csr_wdata[7];
              if (csr_wdata[2]) begin
                done_sticky <= 1'b0;
                err_sticky  <= 1'b0;
                err_code    <= `ERR_NONE;
              end
            end
            `CSR_PANEL_SEL:       panel_sel       <= csr_wdata[1:0];
            `CSR_QSPI_CFG: begin
              rd_lat   <= csr_wdata[1:0];
              rd_half  <= csr_wdata[2];
              sck_div  <= csr_wdata[4:3];
              psram_en <= csr_wdata[5];
            end
            `CSR_FLASH_BASE_L:    flash_base[7:0]   <= csr_wdata;
            `CSR_FLASH_BASE_M:    flash_base[15:8]  <= csr_wdata;
            `CSR_FLASH_BASE_H:    flash_base[23:16] <= csr_wdata;
            `CSR_CTX_BASE_L:      ctx_base[7:0]     <= csr_wdata;
            `CSR_CTX_BASE_M:      ctx_base[15:8]    <= csr_wdata;
            `CSR_CTX_BASE_H:      ctx_base[23:16]   <= csr_wdata;
            `CSR_PSRAM_BURST_MAX: psram_burst_max   <= csr_wdata;
            `CSR_INIT_WAIT:       init_wait         <= csr_wdata;
            `CSR_ERR_CODE: begin
              if (csr_wdata == 8'h00) begin
                err_sticky <= 1'b0;
                err_code   <= `ERR_NONE;
              end
            end
            default: ;
          endcase
        end
      end
    end
  end

endmodule
