/*
 * NILM-TT 27-State Microsequencer & Descriptor Engine (Sec. 6.2, 8.6)
 */
`default_nettype none
`include "nilm_pkg.vh"

module nilm_seq (
    input  wire               clk,
    input  wire               rst_n,

    // Control & configuration from CSR / Command Decoder / Pins
    input  wire               soft_rst_pulse,
    input  wire               abort_pulse,
    input  wire               start_pulse,       // CSR CTRL.START
    input  wire               run_pin_Sync,      // Rising edge of ui_in[4] -> RUN_ALL
    input  wire               cmd_init_trig,
    input  wire               cmd_run_trig,
    input  wire               cmd_run_all_trig,
    input  wire               cmd_step_trig,
    input  wire        [1:0]  cmd_panel_arg,
    input  wire        [1:0]  csr_panel_sel,
    input  wire               cmd_mem_rd_trig,
    input  wire               cmd_mem_wr_trig,
    input  wire               cmd_mem_dev,
    input  wire        [23:0] cmd_mem_addr,
    input  wire        [7:0]  cmd_mem_len,

    input  wire               deb_en,
    input  wire               ctx_mode,          // 0: PSRAM context, 1: Host context
    input  wire               host_ctx_loaded,
    input  wire               step_mode,         // 1: Pause after one LSTM unit on STEP_UNIT
    input  wire               psram_en,
    input  wire        [23:0] flash_base,
    input  wire        [23:0] ctx_base,
    input  wire        [7:0]  init_wait,
    input  wire               bus_gnt,

    // Status & debug outputs to CSR
    output wire               seq_busy,
    output wire               seq_idle,
    output reg                seq_done_pulse,
    output reg                seq_err_pulse,
    output reg         [7:0]  seq_err_code,
    output reg                qspi_ready,
    output reg         [23:0] cycle_cnt,
    output wire        [4:0]  dbg_state,
    output wire        [4:0]  dbg_unit,

    // Per-panel results & NHEAD
    output reg         [15:0] panel_res_0,
    output reg         [15:0] panel_res_1,
    output reg         [15:0] panel_res_2,
    output reg         [15:0] panel_res_3,
    output reg         [4:0]  panel_nhead_0,
    output reg         [4:0]  panel_nhead_1,
    output reg         [4:0]  panel_nhead_2,
    output reg         [4:0]  panel_nhead_3,

    // QSPI Controller interface
    output reg                qspi_cmd_valid,
    output reg         [1:0]  qspi_cmd_type,     // 1:INIT, 2:READ, 3:WRITE
    output reg                qspi_cmd_dev,      // 0:Flash, 1:PSRAM
    output reg         [23:0] qspi_cmd_addr,
    output reg         [15:0] qspi_cmd_len,
    output reg                qspi_cmd_abort,
    input  wire               qspi_cmd_busy,
    input  wire               qspi_cmd_done,
    input  wire               qspi_rd_valid,
    input  wire        [7:0]  qspi_rd_data,
    output wire               qspi_rd_ready,
    input  wire               qspi_wr_req,
    output reg         [7:0]  qspi_wr_data,

    // MAC & PWL & Cell & Head datapath interfaces
    output reg                mac_clr_sat,
    output reg                mac_load_bias,
    output reg                mac_en,
    output reg  signed [15:0] mac_bias_in,
    output reg         [3:0]  mac_bias_shift,
    output reg  signed [7:0]  mac_w_in,
    output reg  signed [7:0]  mac_u_in,
    output reg         [3:0]  mac_req_shift,
    input  wire signed [7:0]  mac_req8_out,
    input  wire               mac_req8_sat,
    input  wire signed [15:0] mac_req16_out,
    input  wire               mac_req16_sat,

    output wire signed [7:0]  pwl_pre_in,
    input  wire        [7:0]  pwl_sigma_out,
    input  wire signed [7:0]  pwl_tanh_out,

    output wire               cell_type_out,
    output reg         [7:0]  gate_a_i,
    output reg         [7:0]  gate_a_f,
    output reg  signed [7:0]  gate_a_g,
    output reg         [7:0]  gate_a_o,
    input  wire signed [15:0] cell_c_new,
    input  wire signed [7:0]  cell_h_new,
    input  wire signed [7:0]  cell_gru_rh,

    output reg  signed [15:0] head_logit_in,
    output reg  signed [15:0] head_theta_in,
    output wire        [3:0]  head_deb_n,
    output wire        [3:0]  head_deb_m,
    input  wire        [3:0]  head_deb_hist_out,
    input  wire               head_on_out,

    // VRAM ports
    output reg                vram_u_we,
    output reg         [5:0]  vram_u_waddr,
    output reg  signed [7:0]  vram_u_wdata,
    output reg         [5:0]  vram_u_raddr,
    input  wire signed [7:0]  vram_u_rdata,

    output reg                vram_hb_we,
    output reg         [4:0]  vram_hb_waddr,
    output reg  signed [7:0]  vram_hb_wdata,
    output reg         [4:0]  vram_hb_raddr,
    input  wire signed [7:0]  vram_hb_rdata,

    output reg                vram_h_commit,
    output wire        [5:0]  vram_f_dim,
    output wire        [5:0]  vram_h_dim,

    output reg                vram_c_we,
    output reg         [4:0]  vram_c_waddr,
    output reg  signed [15:0] vram_c_wdata,
    output reg         [4:0]  vram_c_raddr,
    input  wire signed [15:0] vram_c_rdata,

    output reg                vram_deb_we,
    output reg         [3:0]  vram_deb_waddr,
    output reg         [3:0]  vram_deb_wdata,
    output reg         [3:0]  vram_deb_raddr,
    input  wire        [3:0]  vram_deb_rdata,
    output reg         [1:0]  vram_deb_pair_idx,
    input  wire        [7:0]  vram_deb_pair_rdata,

    output reg                vram_logit_we,
    output reg         [3:0]  vram_logit_waddr,
    output reg  signed [15:0] vram_logit_wdata
);

  reg [4:0] state;
  assign dbg_state = state;
  assign seq_idle  = (state == `ST_IDLE) || (state == `ST_BUS_HOST);
  assign seq_busy  = !seq_idle;

  reg run_pin_d;
  wire run_pin_rise = run_pin_Sync && (!run_pin_d);

  reg [2:0]  blob_npanel;
  reg [23:0] desc_off_0;
  reg [23:0] desc_off_1;
  reg [23:0] desc_off_2;
  reg [23:0] desc_off_3;

  reg [1:0]  cur_panel;
  reg        run_all_active;
  reg        single_unit_run;
  reg        mem_op_active;
  reg        mem_op_write;

  reg [5:0]  desc_f;
  reg [5:0]  desc_h;
  reg [4:0]  desc_nz;
  reg [4:0]  desc_nhead;
  reg        desc_cell_type;
  reg [3:0]  desc_s_i, desc_b_i;
  reg [3:0]  desc_s_f, desc_b_f;
  reg [3:0]  desc_s_g, desc_b_g;
  reg [3:0]  desc_s_o, desc_b_o;
  reg [3:0]  desc_s_z, desc_b_z;
  reg [3:0]  desc_s_head, desc_b_head;
  reg [3:0]  desc_deb_n, desc_deb_m;
  reg [23:0] desc_w_lstm_off;
  reg [23:0] desc_w_z_off;
  reg [23:0] desc_w_head_off;

  assign vram_f_dim    = desc_f;
  assign vram_h_dim    = desc_h;
  assign cell_type_out = desc_cell_type;
  assign head_deb_n    = desc_deb_n;
  assign head_deb_m    = desc_deb_m;

  reg [4:0]  unit_j;
  reg [1:0]  gate_k;
  reg [6:0]  elem_n;
  reg [4:0]  dense_l;
  reg [4:0]  head_m;
  reg [15:0] sub_cnt;
  reg [7:0]  byte_lo;
  reg [7:0]  theta_lo;
  reg        qspi_started;
  reg [15:0] accum_res_bits;
  reg        deb_hi_sel;

  assign dbg_unit   = unit_j;
  assign pwl_pre_in = mac_req8_out;

  reg [3:0] cur_gate_s;
  reg [3:0] cur_gate_b;
  always @(*) begin
    case (gate_k)
      2'd0: begin cur_gate_s = desc_s_i; cur_gate_b = desc_b_i; end
      2'd1: begin cur_gate_s = desc_s_f; cur_gate_b = desc_b_f; end
      2'd2: begin cur_gate_s = desc_s_g; cur_gate_b = desc_b_g; end
      default: begin cur_gate_s = desc_s_o; cur_gate_b = desc_b_o; end
    endcase
  end

  wire [1:0]  num_gates_m1 = desc_cell_type ? 2'd2 : 2'd3;
  wire [6:0]  vec_len      = {1'b0, desc_f} + {1'b0, desc_h};
  wire [15:0] gates_per_u  = desc_cell_type ? 16'd3 : 16'd4;
  wire [15:0] gate_stride  = 16'd2 + {9'd0, vec_len};
  wire [15:0] unit_stride  = gates_per_u * gate_stride;
  wire [23:0] unit_off_24  = {19'd0, unit_j} * {8'd0, unit_stride};

  reg [23:0] cur_desc_off;
  always @(*) begin
    case (cur_panel)
      2'd0: cur_desc_off = desc_off_0;
      2'd1: cur_desc_off = desc_off_1;
      2'd2: cur_desc_off = desc_off_2;
      default: cur_desc_off = desc_off_3;
    endcase
  end

  assign qspi_rd_ready = (state == `ST_CHK_MAGIC)  ||
                         (state == `ST_DESC_FETCH) ||
                         (state == `ST_CTX_LOAD)   ||
                         (state == `ST_GATE_BIAS)  ||
                         (state == `ST_GATE_MAC)   ||
                         (state == `ST_DENSE_BIAS) ||
                         (state == `ST_DENSE_MAC)  ||
                         (state == `ST_HEAD_BIAS)  ||
                         (state == `ST_HEAD_MAC);

  wire [15:0] c_byte_off   = (sub_cnt - 16'd32) >> 1;
  wire [15:0] deb_byte_off = sub_cnt - 16'd96;
  wire [15:0] deb_hi_off   = sub_cnt - 16'd97;

  // Combinational VRAM read address steering
  always @(*) begin
    vram_u_raddr      = 6'd0;
    vram_hb_raddr     = 5'd0;
    vram_c_raddr      = unit_j;
    vram_deb_raddr    = head_m[3:0];
    vram_deb_pair_idx = 2'd0;

    if (state == `ST_GATE_MAC) begin
      if (elem_n >= {1'b0, desc_f}) begin
        vram_u_raddr = 6'd32 + (elem_n[5:0] - desc_f);
        vram_c_raddr = elem_n[4:0] - desc_f[4:0];
      end else begin
        vram_u_raddr = elem_n[5:0];
      end
    end else if (state == `ST_DENSE_MAC) begin
      vram_u_raddr = 6'd32 + elem_n[5:0];
    end else if (state == `ST_HEAD_MAC) begin
      vram_hb_raddr = elem_n[4:0];
    end else if (state == `ST_CTX_SAVE) begin
      if (mem_op_active) begin
        vram_u_raddr = sub_cnt[5:0];
      end else if (sub_cnt < 16'd32) begin
        vram_u_raddr = 6'd32 + sub_cnt[5:0];
      end else if (sub_cnt < 16'd96) begin
        vram_c_raddr = c_byte_off[4:0];
      end else begin
        vram_deb_pair_idx = deb_byte_off[1:0];
      end
    end
  end

  // Combinational write byte for PSRAM CTX_SAVE or MEM_WR
  always @(*) begin
    if (mem_op_active) begin
      qspi_wr_data = vram_u_rdata;
    end else if (sub_cnt < 16'd32) begin
      qspi_wr_data = (sub_cnt[5:0] < desc_h) ? vram_u_rdata : 8'h00;
    end else if (sub_cnt < 16'd96) begin
      qspi_wr_data = sub_cnt[0] ? vram_c_rdata[15:8] : vram_c_rdata[7:0];
    end else begin
      qspi_wr_data = vram_deb_pair_rdata;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state            <= `ST_IDLE;
      run_pin_d        <= 1'b0;
      seq_done_pulse   <= 1'b0;
      seq_err_pulse    <= 1'b0;
      seq_err_code     <= `ERR_NONE;
      qspi_ready       <= 1'b0;
      cycle_cnt        <= 24'd0;
      blob_npanel      <= 3'd3;
      desc_off_0       <= 24'd32;
      desc_off_1       <= 24'd96;
      desc_off_2       <= 24'd160;
      desc_off_3       <= 24'd224;
      cur_panel        <= 2'd0;
      run_all_active   <= 1'b0;
      single_unit_run  <= 1'b0;
      mem_op_active    <= 1'b0;
      mem_op_write     <= 1'b0;
      desc_f           <= 6'd14;
      desc_h           <= 6'd32;
      desc_nz          <= 5'd16;
      desc_nhead       <= 5'd4;
      desc_cell_type   <= 1'b0;
      desc_s_i         <= 4'd7; desc_b_i    <= 4'd0;
      desc_s_f         <= 4'd7; desc_b_f    <= 4'd0;
      desc_s_g         <= 4'd7; desc_b_g    <= 4'd0;
      desc_s_o         <= 4'd7; desc_b_o    <= 4'd0;
      desc_s_z         <= 4'd7; desc_b_z    <= 4'd0;
      desc_s_head      <= 4'd4; desc_b_head <= 4'd0;
      desc_deb_n       <= 4'd0; desc_deb_m  <= 4'd1;
      desc_w_lstm_off  <= 24'd0;
      desc_w_z_off     <= 24'd0;
      desc_w_head_off  <= 24'd0;
      unit_j           <= 5'd0;
      gate_k           <= 2'd0;
      elem_n           <= 7'd0;
      dense_l          <= 5'd0;
      head_m           <= 5'd0;
      sub_cnt          <= 16'd0;
      byte_lo          <= 8'h00;
      theta_lo         <= 8'h00;
      qspi_started     <= 1'b0;
      accum_res_bits   <= 16'h0000;
      panel_res_0      <= 16'h0000;
      panel_res_1      <= 16'h0000;
      panel_res_2      <= 16'h0000;
      panel_res_3      <= 16'h0000;
      panel_nhead_0    <= 5'd4;
      panel_nhead_1    <= 5'd4;
      panel_nhead_2    <= 5'd14;
      panel_nhead_3    <= 5'd4;
      qspi_cmd_valid   <= 1'b0;
      qspi_cmd_type    <= 2'd0;
      qspi_cmd_dev     <= 1'b0;
      qspi_cmd_addr    <= 24'd0;
      qspi_cmd_len     <= 16'd0;
      qspi_cmd_abort   <= 1'b0;
      mac_clr_sat      <= 1'b0;
      mac_load_bias    <= 1'b0;
      mac_en           <= 1'b0;
      mac_bias_in      <= 16'sd0;
      mac_bias_shift   <= 4'd0;
      mac_w_in         <= 8'sd0;
      mac_u_in         <= 8'sd0;
      mac_req_shift    <= 4'd0;
      gate_a_i         <= 8'd0;
      gate_a_f         <= 8'd0;
      gate_a_g         <= 8'sd0;
      gate_a_o         <= 8'd0;
      head_logit_in    <= 16'sd0;
      head_theta_in    <= 16'sd0;
      vram_u_we        <= 1'b0;
      vram_u_waddr     <= 6'd0;
      vram_u_wdata     <= 8'sd0;
      vram_hb_we       <= 1'b0;
      vram_hb_waddr    <= 5'd0;
      vram_hb_wdata    <= 8'sd0;
      vram_h_commit    <= 1'b0;
      vram_c_we        <= 1'b0;
      vram_c_waddr     <= 5'd0;
      vram_c_wdata     <= 16'sd0;
      vram_deb_we      <= 1'b0;
      vram_deb_waddr   <= 4'd0;
      vram_deb_wdata   <= 4'd0;
      vram_logit_we    <= 1'b0;
      vram_logit_waddr <= 4'd0;
      vram_logit_wdata <= 16'sd0;
      deb_hi_sel       <= 1'b0;
    end else begin
      run_pin_d      <= run_pin_Sync;
      seq_done_pulse <= 1'b0;
      seq_err_pulse  <= 1'b0;
      qspi_cmd_valid <= 1'b0;
      qspi_cmd_abort <= 1'b0;
      mac_clr_sat    <= 1'b0;
      mac_load_bias  <= 1'b0;
      mac_en         <= 1'b0;
      vram_u_we      <= 1'b0;
      vram_hb_we     <= 1'b0;
      vram_h_commit  <= 1'b0;
      vram_c_we      <= 1'b0;
      vram_deb_we    <= 1'b0;
      vram_logit_we  <= 1'b0;

      if (soft_rst_pulse) begin
        state          <= `ST_IDLE;
        qspi_cmd_abort <= 1'b1;
        qspi_ready     <= 1'b0;
        unit_j         <= 5'd0;
      end else if (abort_pulse && (state != `ST_IDLE) && (state != `ST_BUS_HOST)) begin
        qspi_cmd_abort <= 1'b1;
        seq_err_pulse  <= 1'b1;
        seq_err_code   <= `ERR_ABORT;
        state          <= `ST_ERR;
      end else begin
        if (seq_busy) begin
          cycle_cnt <= cycle_cnt + 24'd1;
        end

        case (state)
          `ST_IDLE, `ST_BUS_HOST: begin
            if (bus_gnt) begin
              state <= `ST_BUS_HOST;
            end else begin
              state <= `ST_IDLE;
              if (cmd_init_trig) begin
                cycle_cnt      <= 24'd0;
                qspi_ready     <= 1'b0;
                qspi_cmd_valid <= 1'b1;
                qspi_cmd_type  <= 2'd1;
                qspi_started   <= 1'b1;
                state          <= `ST_INIT_RST;
              end else if (cmd_run_trig || start_pulse) begin
                cycle_cnt       <= 24'd0;
                mac_clr_sat     <= 1'b1;
                cur_panel       <= cmd_run_trig ? cmd_panel_arg : csr_panel_sel;
                run_all_active  <= 1'b0;
                single_unit_run <= step_mode;
                mem_op_active   <= 1'b0;
                unit_j          <= 5'd0;
                qspi_started    <= 1'b0;
                state           <= `ST_DESC_FETCH;
              end else if (cmd_run_all_trig || run_pin_rise) begin
                cycle_cnt       <= 24'd0;
                mac_clr_sat     <= 1'b1;
                cur_panel       <= 2'd0;
                run_all_active  <= 1'b1;
                single_unit_run <= 1'b0;
                mem_op_active   <= 1'b0;
                unit_j          <= 5'd0;
                qspi_started    <= 1'b0;
                state           <= `ST_DESC_FETCH;
              end else if (cmd_step_trig) begin
                cycle_cnt       <= 24'd0;
                cur_panel       <= csr_panel_sel;
                run_all_active  <= 1'b0;
                single_unit_run <= 1'b1;
                mem_op_active   <= 1'b0;
                qspi_started    <= 1'b0;
                if (unit_j == 5'd0) begin
                  state <= `ST_DESC_FETCH;
                end else begin
                  gate_k         <= 2'd0;
                  elem_n         <= 7'd0;
                  sub_cnt        <= 16'd0;
                  qspi_cmd_valid <= 1'b1;
                  qspi_cmd_type  <= 2'd2;
                  qspi_cmd_dev   <= 1'b0;
                  qspi_cmd_addr  <= flash_base + desc_w_lstm_off + unit_off_24;
                  qspi_cmd_len   <= unit_stride;
                  state          <= `ST_GATE_BIAS;
                end
              end else if (cmd_mem_rd_trig) begin
                cycle_cnt      <= 24'd0;
                mem_op_active  <= 1'b1;
                mem_op_write   <= 1'b0;
                sub_cnt        <= 16'd0;
                qspi_cmd_valid <= 1'b1;
                qspi_cmd_type  <= 2'd2;
                qspi_cmd_dev   <= cmd_mem_dev;
                qspi_cmd_addr  <= cmd_mem_addr;
                qspi_cmd_len   <= {8'd0, cmd_mem_len};
                state          <= `ST_CTX_LOAD;
              end else if (cmd_mem_wr_trig) begin
                cycle_cnt      <= 24'd0;
                mem_op_active  <= 1'b1;
                mem_op_write   <= 1'b1;
                sub_cnt        <= 16'd0;
                qspi_cmd_valid <= 1'b1;
                qspi_cmd_type  <= 2'd3;
                qspi_cmd_dev   <= 1'b1;
                qspi_cmd_addr  <= cmd_mem_addr;
                qspi_cmd_len   <= {8'd0, cmd_mem_len};
                state          <= `ST_CTX_SAVE;
              end
            end
          end

          `ST_INIT_RST: begin
            if (qspi_cmd_done) begin
              sub_cnt <= 16'd0;
              state   <= `ST_INIT_WAIT;
            end
          end

          `ST_INIT_WAIT: begin
            if (sub_cnt >= ({8'd0, init_wait} << 4)) begin
              sub_cnt        <= 16'd0;
              qspi_cmd_valid <= 1'b1;
              qspi_cmd_type  <= 2'd2;
              qspi_cmd_dev   <= 1'b0;
              qspi_cmd_addr  <= flash_base;
              qspi_cmd_len   <= 16'd20;
              state          <= `ST_CHK_MAGIC;
            end else begin
              sub_cnt <= sub_cnt + 16'd1;
            end
          end

          `ST_CHK_MAGIC: begin
            if (qspi_rd_valid) begin
              sub_cnt <= sub_cnt + 16'd1;
              case (sub_cnt)
                16'd0: if (qspi_rd_data != 8'h4E) begin
                  qspi_cmd_abort <= 1'b1;
                  seq_err_pulse  <= 1'b1;
                  seq_err_code   <= `ERR_MAGIC;
                  state          <= `ST_ERR;
                end
                16'd1: if (qspi_rd_data != 8'h49) begin
                  qspi_cmd_abort <= 1'b1;
                  seq_err_pulse  <= 1'b1;
                  seq_err_code   <= `ERR_MAGIC;
                  state          <= `ST_ERR;
                end
                16'd2: if (qspi_rd_data != 8'h4C) begin
                  qspi_cmd_abort <= 1'b1;
                  seq_err_pulse  <= 1'b1;
                  seq_err_code   <= `ERR_MAGIC;
                  state          <= `ST_ERR;
                end
                16'd3: if (qspi_rd_data != 8'h4D) begin
                  qspi_cmd_abort <= 1'b1;
                  seq_err_pulse  <= 1'b1;
                  seq_err_code   <= `ERR_MAGIC;
                  state          <= `ST_ERR;
                end
                16'd5:  blob_npanel       <= qspi_rd_data[2:0];
                16'd8:  desc_off_0[7:0]   <= qspi_rd_data;
                16'd9:  desc_off_0[15:8]  <= qspi_rd_data;
                16'd10: desc_off_0[23:16] <= qspi_rd_data;
                16'd11: desc_off_1[7:0]   <= qspi_rd_data;
                16'd12: desc_off_1[15:8]  <= qspi_rd_data;
                16'd13: desc_off_1[23:16] <= qspi_rd_data;
                16'd14: desc_off_2[7:0]   <= qspi_rd_data;
                16'd15: desc_off_2[15:8]  <= qspi_rd_data;
                16'd16: desc_off_2[23:16] <= qspi_rd_data;
                16'd17: desc_off_3[7:0]   <= qspi_rd_data;
                16'd18: desc_off_3[15:8]  <= qspi_rd_data;
                16'd19: desc_off_3[23:16] <= qspi_rd_data;
                default: ;
              endcase
            end
            if (qspi_cmd_done && state == `ST_CHK_MAGIC) begin
              qspi_ready <= 1'b1;
              state      <= `ST_DONE;
            end
          end

          `ST_DESC_FETCH: begin
            if (!qspi_started) begin
              qspi_started   <= 1'b1;
              sub_cnt        <= 16'd0;
              qspi_cmd_valid <= 1'b1;
              qspi_cmd_type  <= 2'd2;
              qspi_cmd_dev   <= 1'b0;
              qspi_cmd_addr  <= flash_base + cur_desc_off;
              qspi_cmd_len   <= 16'd32;
            end else begin
              if (qspi_rd_valid) begin
                sub_cnt <= sub_cnt + 16'd1;
                case (sub_cnt)
                  16'd0: if (qspi_rd_data != 8'h4E) begin
                    qspi_cmd_abort <= 1'b1;
                    seq_err_pulse  <= 1'b1;
                    seq_err_code   <= `ERR_DESC;
                    state          <= `ST_ERR;
                  end
                  16'd1: if (qspi_rd_data != 8'h54) begin
                    qspi_cmd_abort <= 1'b1;
                    seq_err_pulse  <= 1'b1;
                    seq_err_code   <= `ERR_DESC;
                    state          <= `ST_ERR;
                  end
                  16'd2: if (qspi_rd_data != 8'h50) begin
                    qspi_cmd_abort <= 1'b1;
                    seq_err_pulse  <= 1'b1;
                    seq_err_code   <= `ERR_DESC;
                    state          <= `ST_ERR;
                  end
                  16'd3: if (qspi_rd_data != 8'h44) begin
                    qspi_cmd_abort <= 1'b1;
                    seq_err_pulse  <= 1'b1;
                    seq_err_code   <= `ERR_DESC;
                    state          <= `ST_ERR;
                  end
                  16'd4: begin
                    if (qspi_rd_data == 8'd0 || qspi_rd_data > 8'd32) begin
                      qspi_cmd_abort <= 1'b1;
                      seq_err_pulse  <= 1'b1;
                      seq_err_code   <= `ERR_DESC;
                      state          <= `ST_ERR;
                    end
                    desc_f <= qspi_rd_data[5:0];
                  end
                  16'd5: begin
                    if (qspi_rd_data == 8'd0 || qspi_rd_data > 8'd32) begin
                      qspi_cmd_abort <= 1'b1;
                      seq_err_pulse  <= 1'b1;
                      seq_err_code   <= `ERR_DESC;
                      state          <= `ST_ERR;
                    end
                    desc_h <= qspi_rd_data[5:0];
                  end
                  16'd6:  desc_nz        <= qspi_rd_data[4:0];
                  16'd7: begin
                    desc_nhead <= qspi_rd_data[4:0];
                    case (cur_panel)
                      2'd0: panel_nhead_0 <= qspi_rd_data[4:0];
                      2'd1: panel_nhead_1 <= qspi_rd_data[4:0];
                      2'd2: panel_nhead_2 <= qspi_rd_data[4:0];
                      default: panel_nhead_3 <= qspi_rd_data[4:0];
                    endcase
                  end
                  16'd8:  desc_cell_type <= qspi_rd_data[0];
                  16'd9:  begin desc_s_i <= qspi_rd_data[3:0]; desc_b_i <= qspi_rd_data[7:4]; end
                  16'd10: begin desc_s_f <= qspi_rd_data[3:0]; desc_b_f <= qspi_rd_data[7:4]; end
                  16'd11: begin desc_s_g <= qspi_rd_data[3:0]; desc_b_g <= qspi_rd_data[7:4]; end
                  16'd12: begin desc_s_o <= qspi_rd_data[3:0]; desc_b_o <= qspi_rd_data[7:4]; end
                  16'd13: begin desc_s_z <= qspi_rd_data[3:0]; desc_b_z <= qspi_rd_data[7:4]; end
                  16'd14: begin desc_s_head <= qspi_rd_data[3:0]; desc_b_head <= qspi_rd_data[7:4]; end
                  16'd15: begin desc_deb_n <= qspi_rd_data[3:0]; desc_deb_m <= qspi_rd_data[7:4]; end
                  16'd16: desc_w_lstm_off[7:0]   <= qspi_rd_data;
                  16'd17: desc_w_lstm_off[15:8]  <= qspi_rd_data;
                  16'd18: desc_w_lstm_off[23:16] <= qspi_rd_data;
                  16'd19: desc_w_z_off[7:0]      <= qspi_rd_data;
                  16'd20: desc_w_z_off[15:8]     <= qspi_rd_data;
                  16'd21: desc_w_z_off[23:16]    <= qspi_rd_data;
                  16'd22: desc_w_head_off[7:0]   <= qspi_rd_data;
                  16'd23: desc_w_head_off[15:8]  <= qspi_rd_data;
                  16'd24: desc_w_head_off[23:16] <= qspi_rd_data;
                  default: ;
                endcase
              end
              if (qspi_cmd_done && state == `ST_DESC_FETCH) begin
                qspi_started <= 1'b0;
                sub_cnt      <= 16'd0;
                if (ctx_mode || !psram_en) begin
                  if (host_ctx_loaded) begin
                    vram_h_commit <= 1'b1;
                  end
                  unit_j         <= 5'd0;
                  gate_k         <= 2'd0;
                  elem_n         <= 7'd0;
                  qspi_cmd_valid <= 1'b1;
                  qspi_cmd_type  <= 2'd2;
                  qspi_cmd_dev   <= 1'b0;
                  qspi_cmd_addr  <= flash_base + desc_w_lstm_off;
                  qspi_cmd_len   <= (single_unit_run ? 16'd1 : {10'd0, desc_h}) * unit_stride;
                  state          <= `ST_GATE_BIAS;
                end else begin
                  qspi_cmd_valid <= 1'b1;
                  qspi_cmd_type  <= 2'd2;
                  qspi_cmd_dev   <= 1'b1;
                  qspi_cmd_addr  <= ctx_base + ({14'd0, cur_panel} << 8);
                  qspi_cmd_len   <= 16'd100;
                  state          <= `ST_CTX_LOAD;
                end
              end
            end
          end

          `ST_CTX_LOAD: begin
            if (mem_op_active) begin
              if (qspi_rd_valid) begin
                vram_u_we    <= 1'b1;
                vram_u_waddr <= sub_cnt[5:0];
                vram_u_wdata <= $signed(qspi_rd_data);
                sub_cnt      <= sub_cnt + 16'd1;
              end
              if (qspi_cmd_done) begin
                mem_op_active <= 1'b0;
                state         <= `ST_DONE;
              end
            end else begin
              if (deb_hi_sel) begin
                deb_hi_sel     <= 1'b0;
                vram_deb_we    <= 1'b1;
                vram_deb_waddr <= {deb_hi_off[2:0], 1'b1};
                vram_deb_wdata <= byte_lo[7:4];
              end
              if (qspi_rd_valid) begin
                sub_cnt <= sub_cnt + 16'd1;
                if (sub_cnt < 16'd32) begin
                  if (sub_cnt[5:0] < desc_h) begin
                    vram_u_we    <= 1'b1;
                    vram_u_waddr <= 6'd32 + sub_cnt[5:0];
                    vram_u_wdata <= $signed(qspi_rd_data);
                  end
                end else if (sub_cnt < 16'd96) begin
                  if (sub_cnt[0] == 1'b0) begin
                    byte_lo <= qspi_rd_data;
                  end else begin
                    vram_c_we    <= 1'b1;
                    vram_c_waddr <= c_byte_off[4:0];
                    vram_c_wdata <= $signed({qspi_rd_data, byte_lo});
                  end
                end else if (sub_cnt < 16'd100) begin
                  byte_lo        <= qspi_rd_data;
                  vram_deb_we    <= 1'b1;
                  vram_deb_waddr <= {deb_byte_off[2:0], 1'b0};
                  vram_deb_wdata <= qspi_rd_data[3:0];
                  deb_hi_sel     <= 1'b1;
                end
              end
              if (qspi_cmd_done) begin
                unit_j         <= 5'd0;
                gate_k         <= 2'd0;
                elem_n         <= 7'd0;
                sub_cnt        <= 16'd0;
                qspi_cmd_valid <= 1'b1;
                qspi_cmd_type  <= 2'd2;
                qspi_cmd_dev   <= 1'b0;
                qspi_cmd_addr  <= flash_base + desc_w_lstm_off;
                qspi_cmd_len   <= (single_unit_run ? 16'd1 : {10'd0, desc_h}) * unit_stride;
                state          <= `ST_GATE_BIAS;
              end
            end
          end

          `ST_GATE_BIAS: begin
            if (qspi_rd_valid) begin
              if (sub_cnt == 16'd0) begin
                byte_lo <= qspi_rd_data;
                sub_cnt <= 16'd1;
              end else begin
                mac_load_bias  <= 1'b1;
                mac_bias_in    <= $signed({qspi_rd_data, byte_lo});
                mac_bias_shift <= cur_gate_b;
                sub_cnt        <= 16'd0;
                elem_n         <= 7'd0;
                state          <= `ST_GATE_MAC;
              end
            end
          end

          `ST_GATE_MAC: begin
            if (qspi_rd_valid) begin
              mac_en   <= 1'b1;
              mac_w_in <= $signed(qspi_rd_data);
              if (desc_cell_type && (gate_k == 2'd2) && (elem_n >= {1'b0, desc_f})) begin
                mac_u_in <= cell_gru_rh;
              end else begin
                mac_u_in <= vram_u_rdata;
              end

              if ((elem_n + 7'd1) >= vec_len) begin
                elem_n        <= 7'd0;
                sub_cnt       <= 16'd0;
                mac_req_shift <= cur_gate_s;
                state         <= `ST_GATE_ACT;
              end else begin
                elem_n <= elem_n + 7'd1;
              end
            end
          end

          `ST_GATE_ACT: begin
            if (sub_cnt == 16'd0) begin
              sub_cnt <= 16'd1;
            end else begin
              sub_cnt <= 16'd0;
              case (gate_k)
                2'd0: gate_a_i <= pwl_sigma_out;
                2'd1: gate_a_f <= pwl_sigma_out;
                2'd2: gate_a_g <= pwl_tanh_out;
                default: gate_a_o <= pwl_sigma_out;
              endcase

              if (gate_k == num_gates_m1) begin
                gate_k <= 2'd0;
                state  <= `ST_CELL_FC;
              end else begin
                gate_k <= gate_k + 2'd1;
                state  <= `ST_GATE_BIAS;
              end
            end
          end

          `ST_CELL_FC:   state <= `ST_CELL_IG;
          `ST_CELL_IG:   state <= `ST_CELL_SUM;
          `ST_CELL_SUM:  state <= `ST_CELL_TANH;
          `ST_CELL_TANH: state <= `ST_CELL_H;

          `ST_CELL_H: begin
            vram_c_we     <= 1'b1;
            vram_c_waddr  <= unit_j;
            vram_c_wdata  <= cell_c_new;
            vram_hb_we    <= 1'b1;
            vram_hb_waddr <= unit_j;
            vram_hb_wdata <= cell_h_new;
            state         <= `ST_UNIT_NEXT;
          end

          `ST_UNIT_NEXT: begin
            if (({1'b0, unit_j} + 6'd1) >= desc_h) begin
              state <= `ST_H_COMMIT;
            end else begin
              unit_j <= unit_j + 5'd1;
              if (single_unit_run) begin
                state <= `ST_DONE;
              end else begin
                gate_k  <= 2'd0;
                sub_cnt <= 16'd0;
                state   <= `ST_GATE_BIAS;
              end
            end
          end

          `ST_H_COMMIT: begin
            vram_h_commit <= 1'b1;
            if (!qspi_cmd_busy) begin
              dense_l        <= 5'd0;
              elem_n         <= 7'd0;
              sub_cnt        <= 16'd0;
              qspi_cmd_valid <= 1'b1;
              qspi_cmd_type  <= 2'd2;
              qspi_cmd_dev   <= 1'b0;
              qspi_cmd_addr  <= flash_base + desc_w_z_off;
              qspi_cmd_len   <= {11'd0, desc_nz} * (16'd2 + {10'd0, desc_h});
              state          <= `ST_DENSE_BIAS;
            end
          end

          `ST_DENSE_BIAS: begin
            if (qspi_rd_valid) begin
              if (sub_cnt == 16'd0) begin
                byte_lo <= qspi_rd_data;
                sub_cnt <= 16'd1;
              end else begin
                mac_load_bias  <= 1'b1;
                mac_bias_in    <= $signed({qspi_rd_data, byte_lo});
                mac_bias_shift <= desc_b_z;
                sub_cnt        <= 16'd0;
                elem_n         <= 7'd0;
                state          <= `ST_DENSE_MAC;
              end
            end
          end

          `ST_DENSE_MAC: begin
            if (qspi_rd_valid) begin
              mac_en   <= 1'b1;
              mac_w_in <= $signed(qspi_rd_data);
              mac_u_in <= vram_u_rdata;
              if ((elem_n[5:0] + 6'd1) >= desc_h) begin
                elem_n        <= 7'd0;
                sub_cnt       <= 16'd0;
                mac_req_shift <= desc_s_z;
                state         <= `ST_DENSE_ACT;
              end else begin
                elem_n <= elem_n + 7'd1;
              end
            end
          end

          `ST_DENSE_ACT: begin
            if (sub_cnt == 16'd0) begin
              sub_cnt <= 16'd1;
            end else begin
              vram_hb_we    <= 1'b1;
              vram_hb_waddr <= dense_l;
              vram_hb_wdata <= (mac_req8_out > 8'sd0) ? mac_req8_out : 8'sd0;
              if ((dense_l + 5'd1) >= desc_nz) begin
                if (!qspi_cmd_busy) begin
                  head_m         <= 5'd0;
                  elem_n         <= 7'd0;
                  sub_cnt        <= 16'd0;
                  accum_res_bits <= 16'h0000;
                  qspi_cmd_valid <= 1'b1;
                  qspi_cmd_type  <= 2'd2;
                  qspi_cmd_dev   <= 1'b0;
                  qspi_cmd_addr  <= flash_base + desc_w_head_off;
                  qspi_cmd_len   <= {11'd0, desc_nhead} * (16'd4 + {11'd0, desc_nz});
                  state          <= `ST_HEAD_BIAS;
                end
              end else begin
                dense_l <= dense_l + 5'd1;
                sub_cnt <= 16'd0;
                state   <= `ST_DENSE_BIAS;
              end
            end
          end

          `ST_HEAD_BIAS: begin
            if (qspi_rd_valid) begin
              case (sub_cnt)
                16'd0: begin
                  byte_lo <= qspi_rd_data;
                  sub_cnt <= 16'd1;
                end
                16'd1: begin
                  mac_load_bias  <= 1'b1;
                  mac_bias_in    <= $signed({qspi_rd_data, byte_lo});
                  mac_bias_shift <= desc_b_head;
                  sub_cnt        <= 16'd2;
                end
                16'd2: begin
                  theta_lo <= qspi_rd_data;
                  sub_cnt  <= 16'd3;
                end
                default: begin
                  head_theta_in <= $signed({qspi_rd_data, theta_lo});
                  sub_cnt       <= 16'd0;
                  elem_n        <= 7'd0;
                  state         <= `ST_HEAD_MAC;
                end
              endcase
            end
          end

          `ST_HEAD_MAC: begin
            if (qspi_rd_valid) begin
              mac_en   <= 1'b1;
              mac_w_in <= $signed(qspi_rd_data);
              mac_u_in <= vram_hb_rdata;
              if ((elem_n[4:0] + 5'd1) >= desc_nz) begin
                elem_n        <= 7'd0;
                sub_cnt       <= 16'd0;
                mac_req_shift <= desc_s_head;
                state         <= `ST_HEAD_CMP;
              end else begin
                elem_n <= elem_n + 7'd1;
              end
            end
          end

          `ST_HEAD_CMP: begin
            if (sub_cnt == 16'd0) begin
              sub_cnt <= 16'd1;
            end else begin
              sub_cnt          <= 16'd0;
              head_logit_in    <= mac_req16_out;
              vram_logit_we    <= 1'b1;
              vram_logit_waddr <= head_m[3:0];
              vram_logit_wdata <= mac_req16_out;
              state            <= `ST_DEBOUNCE;
            end
          end

          `ST_DEBOUNCE: begin
            if (sub_cnt == 16'd0) begin
              vram_deb_we    <= 1'b1;
              vram_deb_waddr <= head_m[3:0];
              vram_deb_wdata <= head_deb_hist_out;
              if (head_on_out) begin
                accum_res_bits <= accum_res_bits | (16'd1 << head_m[3:0]);
              end
              if ((head_m + 5'd1) >= desc_nhead) begin
                case (cur_panel)
                  2'd0: panel_res_0 <= accum_res_bits | (head_on_out ? (16'd1 << head_m[3:0]) : 16'd0);
                  2'd1: panel_res_1 <= accum_res_bits | (head_on_out ? (16'd1 << head_m[3:0]) : 16'd0);
                  2'd2: panel_res_2 <= accum_res_bits | (head_on_out ? (16'd1 << head_m[3:0]) : 16'd0);
                  default: panel_res_3 <= accum_res_bits | (head_on_out ? (16'd1 << head_m[3:0]) : 16'd0);
                endcase
              end
            end

            if ((head_m + 5'd1) >= desc_nhead) begin
              if (!qspi_cmd_busy) begin
                unit_j  <= 5'd0;
                sub_cnt <= 16'd0;
                if (ctx_mode || !psram_en) begin
                  state <= `ST_DONE;
                end else begin
                  qspi_cmd_valid <= 1'b1;
                  qspi_cmd_type  <= 2'd3;
                  qspi_cmd_dev   <= 1'b1;
                  qspi_cmd_addr  <= ctx_base + ({14'd0, cur_panel} << 8);
                  qspi_cmd_len   <= 16'd100;
                  state          <= `ST_CTX_SAVE;
                end
              end else begin
                sub_cnt <= 16'd1;
              end
            end else begin
              head_m  <= head_m + 5'd1;
              sub_cnt <= 16'd0;
              state   <= `ST_HEAD_BIAS;
            end
          end

          `ST_CTX_SAVE: begin
            if (qspi_wr_req) begin
              sub_cnt <= sub_cnt + 16'd1;
            end
            if (qspi_cmd_done) begin
              mem_op_active <= 1'b0;
              mem_op_write  <= 1'b0;
              state         <= `ST_DONE;
            end
          end

          `ST_DONE: begin
            if (run_all_active && ({1'b0, cur_panel} + 3'd1 < blob_npanel)) begin
              cur_panel    <= cur_panel + 2'd1;
              unit_j       <= 5'd0;
              qspi_started <= 1'b0;
              state        <= `ST_DESC_FETCH;
            end else begin
              run_all_active <= 1'b0;
              seq_done_pulse <= 1'b1;
              state          <= `ST_IDLE;
            end
          end

          `ST_ERR: begin
            run_all_active <= 1'b0;
            mem_op_active  <= 1'b0;
            if (!qspi_cmd_busy) begin
              state <= `ST_IDLE;
            end
          end

          default: state <= `ST_IDLE;
        endcase
      end
    end
  end

endmodule
