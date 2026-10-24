/*
 * NILM-TT: A Streaming LSTM Load-Disaggregation Engine for Tiny Tapeout
 * Top-level module: tt_um_harsh_nilm (§6.1, §7.1)
 */
`default_nettype none
`include "nilm_pkg.vh"

module tt_um_harsh_nilm (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  reg [1:0] bus_req_sync;
  reg [1:0] run_pin_sync;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bus_req_sync <= 2'b00;
      run_pin_sync <= 2'b00;
    end else begin
      bus_req_sync <= {bus_req_sync[0], ui_in[3]};
      run_pin_sync <= {run_pin_sync[0], ui_in[4]};
    end
  end

  wire       spi_miso;
  wire       frame_start;
  wire       frame_end;
  wire       frame_err;
  wire       rx_valid;
  wire [7:0] rx_byte;
  wire [7:0] tx_byte;

  nilm_host_spi u_host_spi (
      .clk        (clk),
      .rst_n      (rst_n),
      .spi_cs_n   (ui_in[0]),
      .spi_sck    (ui_in[1]),
      .spi_mosi   (ui_in[2]),
      .spi_miso   (spi_miso),
      .frame_start(frame_start),
      .frame_end  (frame_end),
      .frame_err  (frame_err),
      .rx_valid   (rx_valid),
      .rx_byte    (rx_byte),
      .tx_byte    (tx_byte)
  );

  wire        csr_we;
  wire        csr_re;
  wire [7:0]  csr_addr;
  wire [7:0]  csr_wdata;
  wire [7:0]  csr_rdata;

  wire        cmd_err_pulse;
  wire [7:0]  cmd_err_code;
  wire        cmd_bus_rel;
  wire        cmd_bus_acq;

  wire        seq_busy;
  wire        seq_idle;
  wire        seq_done_pulse;
  wire        seq_err_pulse;
  wire [7:0]  seq_err_code;
  wire        bus_gnt;
  wire        bus_owned;
  wire        qspi_ready;
  wire        sat_seen;
  wire [23:0] last_cycles;
  wire [4:0]  dbg_state;
  wire [4:0]  dbg_unit;
  wire signed [23:0] dbg_acc;

  wire [15:0] panel_res_0;
  wire [15:0] panel_res_1;
  wire [15:0] panel_res_2;
  wire [15:0] panel_res_3;
  wire [4:0]  panel_nhead_0;
  wire [4:0]  panel_nhead_1;
  wire [4:0]  panel_nhead_2;
  wire [4:0]  panel_nhead_3;

  wire        done_sticky;
  wire        err_sticky;
  wire [7:0]  err_code;
  wire        ctrl_start_pulse;
  wire        ctrl_abort_pulse;
  wire        ctrl_soft_rst_pulse;
  wire        irq_en;
  wire        deb_en;
  wire        crc_en;
  wire        ctx_mode;
  wire        step_mode;
  wire [1:0]  panel_sel;
  wire [1:0]  rd_lat;
  wire        rd_half;
  wire [1:0]  sck_div;
  wire        psram_en;
  wire [23:0] flash_base;
  wire [23:0] ctx_base;
  wire [7:0]  psram_burst_max;
  wire [7:0]  init_wait;
  wire        host_bus_req;

  wire               win_u_we;
  wire        [5:0]  win_u_addr;
  wire signed [7:0]  win_u_wdata;
  wire               win_hb_we;
  wire        [4:0]  win_hb_addr;
  wire signed [7:0]  win_hb_wdata;
  wire               win_c_we;
  wire        [4:0]  win_c_addr;
  wire signed [15:0] win_c_wdata;

  wire               host_ctx_clear;
  wire               host_ctx_loaded;
  wire               host_u_we;
  wire        [5:0]  host_u_waddr;
  wire signed [7:0]  host_u_wdata;
  wire        [5:0]  host_u_raddr;
  wire               host_hb_we;
  wire        [4:0]  host_hb_waddr;
  wire signed [7:0]  host_hb_wdata;
  wire        [4:0]  host_hb_raddr;
  wire               host_c_we;
  wire        [4:0]  host_c_waddr;
  wire signed [15:0] host_c_wdata;
  wire        [4:0]  host_c_raddr;
  wire               host_deb_we;
  wire        [3:0]  host_deb_waddr;
  wire        [3:0]  host_deb_wdata;
  wire        [1:0]  host_deb_pair_idx;
  wire        [3:0]  logit_raddr;
  wire signed [15:0] logit_rdata;

  wire        cmd_init_trig;
  wire        cmd_run_trig;
  wire        cmd_run_all_trig;
  wire        cmd_step_trig;
  wire [1:0]  cmd_panel_arg;
  wire        cmd_mem_rd_trig;
  wire        cmd_mem_wr_trig;
  wire        cmd_mem_dev;
  wire [23:0] cmd_mem_addr;
  wire [7:0]  cmd_mem_len;

  wire               seq_u_we;
  wire        [5:0]  seq_u_waddr;
  wire signed [7:0]  seq_u_wdata;
  wire        [5:0]  seq_u_raddr;
  wire signed [7:0]  vram_u_rdata;

  wire               seq_hb_we;
  wire        [4:0]  seq_hb_waddr;
  wire signed [7:0]  seq_hb_wdata;
  wire        [4:0]  seq_hb_raddr;
  wire signed [7:0]  vram_hb_rdata;

  wire               vram_h_commit;
  wire        [5:0]  vram_f_dim;
  wire        [5:0]  vram_h_dim;

  wire               seq_c_we;
  wire        [4:0]  seq_c_waddr;
  wire signed [15:0] seq_c_wdata;
  wire        [4:0]  seq_c_raddr;
  wire signed [15:0] vram_c_rdata;

  wire               seq_deb_we;
  wire        [3:0]  seq_deb_waddr;
  wire        [3:0]  seq_deb_wdata;
  wire        [3:0]  seq_deb_raddr;
  wire        [3:0]  vram_deb_rdata;
  wire        [1:0]  seq_deb_pair_idx;
  wire        [7:0]  vram_deb_pair_rdata;

  wire               seq_logit_we;
  wire        [3:0]  seq_logit_waddr;
  wire signed [15:0] seq_logit_wdata;

  nilm_cmd u_cmd (
      .clk                (clk),
      .rst_n              (rst_n),
      .frame_start        (frame_start),
      .frame_end          (frame_end),
      .frame_err          (frame_err),
      .rx_valid           (rx_valid),
      .rx_byte            (rx_byte),
      .tx_byte            (tx_byte),
      .crc_en             (crc_en),
      .seq_busy           (seq_busy),
      .vram_f_dim         (vram_f_dim),
      .csr_we             (csr_we),
      .csr_re             (csr_re),
      .csr_addr           (csr_addr),
      .csr_wdata          (csr_wdata),
      .csr_rdata          (csr_rdata),
      .cmd_err_pulse      (cmd_err_pulse),
      .cmd_err_code       (cmd_err_code),
      .cmd_bus_rel        (cmd_bus_rel),
      .cmd_bus_acq        (cmd_bus_acq),
      .panel_res_0        (panel_res_0),
      .panel_res_1        (panel_res_1),
      .panel_res_2        (panel_res_2),
      .panel_res_3        (panel_res_3),
      .logit_raddr        (logit_raddr),
      .logit_rdata        (logit_rdata),
      .host_ctx_clear     (host_ctx_clear),
      .host_ctx_loaded    (host_ctx_loaded),
      .seq_done_pulse     (seq_done_pulse),
      .host_u_we          (host_u_we),
      .host_u_waddr       (host_u_waddr),
      .host_u_wdata       (host_u_wdata),
      .host_u_raddr       (host_u_raddr),
      .host_u_rdata       (vram_u_rdata),
      .host_hb_we         (host_hb_we),
      .host_hb_waddr      (host_hb_waddr),
      .host_hb_wdata      (host_hb_wdata),
      .host_hb_raddr      (host_hb_raddr),
      .host_hb_rdata      (vram_hb_rdata),
      .host_c_we          (host_c_we),
      .host_c_waddr       (host_c_waddr),
      .host_c_wdata       (host_c_wdata),
      .host_c_raddr       (host_c_raddr),
      .host_c_rdata       (vram_c_rdata),
      .host_deb_we        (host_deb_we),
      .host_deb_waddr     (host_deb_waddr),
      .host_deb_wdata     (host_deb_wdata),
      .host_deb_pair_idx  (host_deb_pair_idx),
      .host_deb_pair_rdata(vram_deb_pair_rdata),
      .cmd_init_trig      (cmd_init_trig),
      .cmd_run_trig       (cmd_run_trig),
      .cmd_run_all_trig   (cmd_run_all_trig),
      .cmd_step_trig      (cmd_step_trig),
      .cmd_panel_arg      (cmd_panel_arg),
      .cmd_mem_rd_trig    (cmd_mem_rd_trig),
      .cmd_mem_wr_trig    (cmd_mem_wr_trig),
      .cmd_mem_dev        (cmd_mem_dev),
      .cmd_mem_addr       (cmd_mem_addr),
      .cmd_mem_len        (cmd_mem_len)
  );

  nilm_csr u_csr (
      .clk                (clk),
      .rst_n              (rst_n),
      .csr_we             (csr_we),
      .csr_re             (csr_re),
      .csr_addr           (csr_addr),
      .csr_wdata          (csr_wdata),
      .csr_rdata          (csr_rdata),
      .cmd_err_pulse      (cmd_err_pulse),
      .cmd_err_code       (cmd_err_code),
      .cmd_bus_rel        (cmd_bus_rel),
      .cmd_bus_acq        (cmd_bus_acq),
      .seq_busy           (seq_busy),
      .seq_done_pulse     (seq_done_pulse),
      .seq_err_pulse      (seq_err_pulse),
      .seq_err_code       (seq_err_code),
      .bus_gnt            (bus_gnt),
      .qspi_ready         (qspi_ready),
      .sat_seen           (sat_seen),
      .last_cycles        (last_cycles),
      .dbg_state          (dbg_state),
      .dbg_unit           (dbg_unit),
      .dbg_acc            (dbg_acc),
      .panel_res_0        (panel_res_0),
      .panel_res_1        (panel_res_1),
      .panel_res_2        (panel_res_2),
      .panel_res_3        (panel_res_3),
      .panel_nhead_0      (panel_nhead_0),
      .panel_nhead_1      (panel_nhead_1),
      .panel_nhead_2      (panel_nhead_2),
      .panel_nhead_3      (panel_nhead_3),
      .win_u_we           (win_u_we),
      .win_u_addr         (win_u_addr),
      .win_u_wdata        (win_u_wdata),
      .win_u_rdata        (vram_u_rdata),
      .win_hb_we          (win_hb_we),
      .win_hb_addr        (win_hb_addr),
      .win_hb_wdata       (win_hb_wdata),
      .win_hb_rdata       (vram_hb_rdata),
      .win_c_we           (win_c_we),
      .win_c_addr         (win_c_addr),
      .win_c_wdata        (win_c_wdata),
      .win_c_rdata        (vram_c_rdata),
      .done_sticky        (done_sticky),
      .err_sticky         (err_sticky),
      .err_code           (err_code),
      .ctrl_start_pulse   (ctrl_start_pulse),
      .ctrl_abort_pulse   (ctrl_abort_pulse),
      .ctrl_soft_rst_pulse(ctrl_soft_rst_pulse),
      .irq_en             (irq_en),
      .deb_en             (deb_en),
      .crc_en             (crc_en),
      .ctx_mode           (ctx_mode),
      .step_mode          (step_mode),
      .panel_sel          (panel_sel),
      .rd_lat             (rd_lat),
      .rd_half            (rd_half),
      .sck_div            (sck_div),
      .psram_en           (psram_en),
      .flash_base         (flash_base),
      .ctx_base           (ctx_base),
      .psram_burst_max    (psram_burst_max),
      .init_wait          (init_wait),
      .host_bus_req       (host_bus_req)
  );

  // Multiplex VRAM ports between Sequencer (when busy) and Host / CSR Window (when idle)
  wire               mux_u_we    = seq_busy ? seq_u_we    : (win_u_we | host_u_we);
  wire        [5:0]  mux_u_waddr = seq_busy ? seq_u_waddr : (win_u_we ? win_u_addr : host_u_waddr);
  wire signed [7:0]  mux_u_wdata = seq_busy ? seq_u_wdata : (win_u_we ? win_u_wdata : host_u_wdata);
  wire        [5:0]  mux_u_raddr = seq_busy ? seq_u_raddr :
                                   ((csr_addr >= 8'h40 && csr_addr <= 8'h7F) ? win_u_addr : host_u_raddr);

  wire               mux_hb_we    = seq_busy ? seq_hb_we    : (win_hb_we | host_hb_we);
  wire        [4:0]  mux_hb_waddr = seq_busy ? seq_hb_waddr : (win_hb_we ? win_hb_addr : host_hb_waddr);
  wire signed [7:0]  mux_hb_wdata = seq_busy ? seq_hb_wdata : (win_hb_we ? win_hb_wdata : host_hb_wdata);
  wire        [4:0]  mux_hb_raddr = seq_busy ? seq_hb_raddr :
                                    ((csr_addr >= 8'h80 && csr_addr <= 8'h9F) ? win_hb_addr : host_hb_raddr);

  wire               mux_c_we    = seq_busy ? seq_c_we    : (win_c_we | host_c_we);
  wire        [4:0]  mux_c_waddr = seq_busy ? seq_c_waddr : (win_c_we ? win_c_addr : host_c_waddr);
  wire signed [15:0] mux_c_wdata = seq_busy ? seq_c_wdata : (win_c_we ? win_c_wdata : host_c_wdata);
  wire        [4:0]  mux_c_raddr = seq_busy ? seq_c_raddr :
                                   ((csr_addr >= 8'hA0 && csr_addr <= 8'hDF) ? win_c_addr : host_c_raddr);

  wire               mux_deb_we       = seq_busy ? seq_deb_we       : host_deb_we;
  wire        [3:0]  mux_deb_waddr    = seq_busy ? seq_deb_waddr    : host_deb_waddr;
  wire        [3:0]  mux_deb_wdata    = seq_busy ? seq_deb_wdata    : host_deb_wdata;
  wire        [3:0]  mux_deb_raddr    = seq_deb_raddr;
  wire        [1:0]  mux_deb_pair_idx = seq_busy ? seq_deb_pair_idx : host_deb_pair_idx;

  nilm_vram u_vram (
      .clk           (clk),
      .rst_n         (rst_n),
      .ctx_clear     (host_ctx_clear | ctrl_soft_rst_pulse),
      .u_we          (mux_u_we),
      .u_waddr       (mux_u_waddr),
      .u_wdata       (mux_u_wdata),
      .u_raddr       (mux_u_raddr),
      .u_rdata       (vram_u_rdata),
      .hb_we         (mux_hb_we),
      .hb_waddr      (mux_hb_waddr),
      .hb_wdata      (mux_hb_wdata),
      .hb_raddr      (mux_hb_raddr),
      .hb_rdata      (vram_hb_rdata),
      .h_commit      (vram_h_commit),
      .c_we          (mux_c_we),
      .c_waddr       (mux_c_waddr),
      .c_wdata       (mux_c_wdata),
      .c_raddr       (mux_c_raddr),
      .c_rdata       (vram_c_rdata),
      .deb_we        (mux_deb_we),
      .deb_waddr     (mux_deb_waddr),
      .deb_wdata     (mux_deb_wdata),
      .deb_raddr     (mux_deb_raddr),
      .deb_rdata     (vram_deb_rdata),
      .deb_pair_idx  (mux_deb_pair_idx),
      .deb_pair_rdata(vram_deb_pair_rdata),
      .logit_we      (seq_logit_we),
      .logit_waddr   (seq_logit_waddr),
      .logit_wdata   (seq_logit_wdata),
      .logit_raddr   (logit_raddr),
      .logit_rdata   (logit_rdata)
  );

  wire        qspi_cmd_valid;
  wire [1:0]  qspi_cmd_type;
  wire        qspi_cmd_dev;
  wire [23:0] qspi_cmd_addr;
  wire [15:0] qspi_cmd_len;
  wire        qspi_cmd_abort;
  wire        qspi_cmd_busy;
  wire        qspi_cmd_done;
  wire        qspi_rd_valid;
  wire [7:0]  qspi_rd_data;
  wire        qspi_rd_ready;
  wire        qspi_wr_req;
  wire [7:0]  qspi_wr_data;

  nilm_qspi u_qspi (
      .clk              (clk),
      .rst_n            (rst_n),
      .sck_div_sel      (sck_div),
      .rd_lat           (rd_lat),
      .rd_half          (rd_half),
      .psram_burst_max  (psram_burst_max),
      .bus_req          (bus_req_sync[1] | host_bus_req),
      .bus_acquire_pulse(cmd_bus_acq),
      .seq_idle         (seq_idle),
      .bus_gnt          (bus_gnt),
      .bus_owned        (bus_owned),
      .cmd_valid        (qspi_cmd_valid),
      .cmd_type         (qspi_cmd_type),
      .cmd_dev          (qspi_cmd_dev),
      .cmd_addr         (qspi_cmd_addr),
      .cmd_len          (qspi_cmd_len),
      .cmd_abort        (qspi_cmd_abort),
      .cmd_busy         (qspi_cmd_busy),
      .cmd_done         (qspi_cmd_done),
      .rd_valid         (qspi_rd_valid),
      .rd_data          (qspi_rd_data),
      .rd_ready         (qspi_rd_ready),
      .wr_req           (qspi_wr_req),
      .wr_data          (qspi_wr_data),
      .uio_in           (uio_in),
      .uio_out          (uio_out),
      .uio_oe           (uio_oe)
  );

  wire               mac_clr_sat;
  wire               mac_load_bias;
  wire               mac_en;
  wire signed [15:0] mac_bias_in;
  wire        [3:0]  mac_bias_shift;
  wire signed [7:0]  mac_w_in;
  wire signed [7:0]  mac_u_in;
  wire        [3:0]  mac_req_shift;
  wire signed [7:0]  mac_req8_out;
  wire               mac_req8_sat;
  wire signed [15:0] mac_req16_out;
  wire               mac_req16_sat;
  wire               mac_sat_seen;

  nilm_mac u_mac (
      .clk       (clk),
      .rst_n     (rst_n),
      .clr_sat   (mac_clr_sat | ctrl_soft_rst_pulse),
      .load_bias (mac_load_bias),
      .mac_en    (mac_en),
      .bias_in   (mac_bias_in),
      .bias_shift(mac_bias_shift),
      .w_in      (mac_w_in),
      .u_in      (mac_u_in),
      .req_shift (mac_req_shift),
      .acc_out   (dbg_acc),
      .req8_out  (mac_req8_out),
      .req8_sat  (mac_req8_sat),
      .req16_out (mac_req16_out),
      .req16_sat (mac_req16_sat),
      .sat_seen  (mac_sat_seen)
  );

  wire signed [7:0] pwl_pre_in;
  wire        [7:0] pwl_sigma_out;
  wire signed [7:0] pwl_tanh_out;

  nilm_pwl u_pwl (
      .pre_in   (pwl_pre_in),
      .sigma_out(pwl_sigma_out),
      .tanh_out (pwl_tanh_out)
  );

  wire               cell_type_out;
  wire        [7:0]  gate_a_i;
  wire        [7:0]  gate_a_f;
  wire signed [7:0]  gate_a_g;
  wire        [7:0]  gate_a_o;
  wire signed [15:0] cell_c_new;
  wire signed [7:0]  cell_h_new;
  wire signed [7:0]  cell_gru_rh;
  wire               cell_sat;

  wire [5:0] cur_h_u_addr = 6'd32 + {1'b0, dbg_unit};
  nilm_cell u_cell (
      .cell_type(cell_type_out),
      .a_i      (gate_a_i),
      .a_f      (gate_a_f),
      .a_g      (gate_a_g),
      .a_o      (gate_a_o),
      .c_old    (vram_c_rdata),
      .h_old    (vram_u_rdata),
      .c_new    (cell_c_new),
      .h_new    (cell_h_new),
      .gru_r_h  (cell_gru_rh),
      .cell_sat (cell_sat)
  );

  reg cell_sat_sticky;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cell_sat_sticky <= 1'b0;
    end else if (mac_clr_sat || ctrl_soft_rst_pulse) begin
      cell_sat_sticky <= 1'b0;
    end else if (cell_sat || mac_req8_sat || mac_req16_sat) begin
      cell_sat_sticky <= 1'b1;
    end
  end
  assign sat_seen = mac_sat_seen | cell_sat_sticky;

  wire signed [15:0] head_logit_in;
  wire signed [15:0] head_theta_in;
  wire        [3:0]  head_deb_n;
  wire        [3:0]  head_deb_m;
  wire               head_raw_on;
  wire        [3:0]  head_deb_hist_out;
  wire               head_on_out;

  nilm_head u_head (
      .logit_in    (head_logit_in),
      .theta_in    (head_theta_in),
      .deb_hist_in (vram_deb_rdata),
      .deb_en      (deb_en),
      .deb_n       (head_deb_n),
      .deb_m       (head_deb_m),
      .raw_on      (head_raw_on),
      .deb_hist_out(head_deb_hist_out),
      .head_on     (head_on_out)
  );

  wire [5:0] seq_u_raddr_raw;
  assign seq_u_raddr = (dbg_state == `ST_CELL_FC || dbg_state == `ST_CELL_IG ||
                        dbg_state == `ST_CELL_SUM || dbg_state == `ST_CELL_TANH ||
                        dbg_state == `ST_CELL_H) ? cur_h_u_addr : seq_u_raddr_raw;

  nilm_seq u_seq (
      .clk                (clk),
      .rst_n              (rst_n),
      .soft_rst_pulse     (ctrl_soft_rst_pulse),
      .abort_pulse        (ctrl_abort_pulse),
      .start_pulse        (ctrl_start_pulse),
      .run_pin_Sync       (run_pin_sync[1]),
      .cmd_init_trig      (cmd_init_trig),
      .cmd_run_trig       (cmd_run_trig),
      .cmd_run_all_trig   (cmd_run_all_trig),
      .cmd_step_trig      (cmd_step_trig),
      .cmd_panel_arg      (cmd_panel_arg),
      .csr_panel_sel      (panel_sel),
      .cmd_mem_rd_trig    (cmd_mem_rd_trig),
      .cmd_mem_wr_trig    (cmd_mem_wr_trig),
      .cmd_mem_dev        (cmd_mem_dev),
      .cmd_mem_addr       (cmd_mem_addr),
      .cmd_mem_len        (cmd_mem_len),
      .deb_en             (deb_en),
      .ctx_mode           (ctx_mode),
      .host_ctx_loaded    (host_ctx_loaded),
      .step_mode          (step_mode),
      .psram_en           (psram_en),
      .flash_base         (flash_base),
      .ctx_base           (ctx_base),
      .init_wait          (init_wait),
      .bus_gnt            (bus_gnt),
      .seq_busy           (seq_busy),
      .seq_idle           (seq_idle),
      .seq_done_pulse     (seq_done_pulse),
      .seq_err_pulse      (seq_err_pulse),
      .seq_err_code       (seq_err_code),
      .qspi_ready         (qspi_ready),
      .cycle_cnt          (last_cycles),
      .dbg_state          (dbg_state),
      .dbg_unit           (dbg_unit),
      .panel_res_0        (panel_res_0),
      .panel_res_1        (panel_res_1),
      .panel_res_2        (panel_res_2),
      .panel_res_3        (panel_res_3),
      .panel_nhead_0      (panel_nhead_0),
      .panel_nhead_1      (panel_nhead_1),
      .panel_nhead_2      (panel_nhead_2),
      .panel_nhead_3      (panel_nhead_3),
      .qspi_cmd_valid     (qspi_cmd_valid),
      .qspi_cmd_type      (qspi_cmd_type),
      .qspi_cmd_dev       (qspi_cmd_dev),
      .qspi_cmd_addr      (qspi_cmd_addr),
      .qspi_cmd_len       (qspi_cmd_len),
      .qspi_cmd_abort     (qspi_cmd_abort),
      .qspi_cmd_busy      (qspi_cmd_busy),
      .qspi_cmd_done      (qspi_cmd_done),
      .qspi_rd_valid      (qspi_rd_valid),
      .qspi_rd_data       (qspi_rd_data),
      .qspi_rd_ready      (qspi_rd_ready),
      .qspi_wr_req        (qspi_wr_req),
      .qspi_wr_data       (qspi_wr_data),
      .mac_clr_sat        (mac_clr_sat),
      .mac_load_bias      (mac_load_bias),
      .mac_en             (mac_en),
      .mac_bias_in        (mac_bias_in),
      .mac_bias_shift     (mac_bias_shift),
      .mac_w_in           (mac_w_in),
      .mac_u_in           (mac_u_in),
      .mac_req_shift      (mac_req_shift),
      .mac_req8_out       (mac_req8_out),
      .mac_req8_sat       (mac_req8_sat),
      .mac_req16_out      (mac_req16_out),
      .mac_req16_sat      (mac_req16_sat),
      .pwl_pre_in         (pwl_pre_in),
      .pwl_sigma_out      (pwl_sigma_out),
      .pwl_tanh_out       (pwl_tanh_out),
      .cell_type_out      (cell_type_out),
      .gate_a_i           (gate_a_i),
      .gate_a_f           (gate_a_f),
      .gate_a_g           (gate_a_g),
      .gate_a_o           (gate_a_o),
      .cell_c_new         (cell_c_new),
      .cell_h_new         (cell_h_new),
      .cell_gru_rh        (cell_gru_rh),
      .head_logit_in      (head_logit_in),
      .head_theta_in      (head_theta_in),
      .head_deb_n         (head_deb_n),
      .head_deb_m         (head_deb_m),
      .head_deb_hist_out  (head_deb_hist_out),
      .head_on_out        (head_on_out),
      .vram_u_we          (seq_u_we),
      .vram_u_waddr       (seq_u_waddr),
      .vram_u_wdata       (seq_u_wdata),
      .vram_u_raddr       (seq_u_raddr_raw),
      .vram_u_rdata       (vram_u_rdata),
      .vram_hb_we         (seq_hb_we),
      .vram_hb_waddr      (seq_hb_waddr),
      .vram_hb_wdata      (seq_hb_wdata),
      .vram_hb_raddr      (seq_hb_raddr),
      .vram_hb_rdata      (vram_hb_rdata),
      .vram_h_commit      (vram_h_commit),
      .vram_f_dim         (vram_f_dim),
      .vram_h_dim         (vram_h_dim),
      .vram_c_we          (seq_c_we),
      .vram_c_waddr       (seq_c_waddr),
      .vram_c_wdata       (seq_c_wdata),
      .vram_c_raddr       (seq_c_raddr),
      .vram_c_rdata       (vram_c_rdata),
      .vram_deb_we        (seq_deb_we),
      .vram_deb_waddr     (seq_deb_waddr),
      .vram_deb_wdata     (seq_deb_wdata),
      .vram_deb_raddr     (seq_deb_raddr),
      .vram_deb_rdata     (vram_deb_rdata),
      .vram_deb_pair_idx  (seq_deb_pair_idx),
      .vram_deb_pair_rdata(vram_deb_pair_rdata),
      .vram_logit_we      (seq_logit_we),
      .vram_logit_waddr   (seq_logit_waddr),
      .vram_logit_wdata   (seq_logit_wdata)
  );

  wire [2:0] dbg_sel = ui_in[7:5];
  reg  [2:0] dbg_pins;
  always @(*) begin
    case (dbg_sel)
      3'd0: dbg_pins = dbg_state[2:0];
      3'd1: dbg_pins = {1'b0, dbg_state[4:3]};
      3'd2: dbg_pins = dbg_unit[2:0];
      3'd3: dbg_pins = panel_res_0[2:0];
      3'd4: dbg_pins = panel_res_1[2:0];
      3'd5: dbg_pins = panel_res_2[2:0];
      3'd6: dbg_pins = {uio_out[3], uio_out[6], uio_out[0]};
      default: dbg_pins = {sat_seen, qspi_ready, bus_owned};
    endcase
  end

  assign uo_out[0]   = spi_miso;
  assign uo_out[1]   = done_sticky & irq_en;
  assign uo_out[2]   = seq_busy;
  assign uo_out[3]   = bus_gnt;
  assign uo_out[4]   = err_sticky;
  assign uo_out[7:5] = dbg_pins;

  wire _unused = &{ena, head_raw_on, 1'b0};

endmodule
