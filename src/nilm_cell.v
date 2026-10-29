/*
 * LSTM & GRU Cell Elementwise Update Unit (Sec. 8.4)
 * Uses a single time-multiplexed 10x16 signed multiplier across cell states:
 *   - cell_step == 2'd0 (ST_CELL_FC):
 *       LSTM: fc_reg <= (a_f * c_old + 128) >>> 8
 *       GRU : fc_reg <= (256 - a_z) * h_old
 *   - cell_step == 2'd1 (ST_CELL_IG):
 *       LSTM: c_new  <= sat16(fc_reg + ((a_i * a_g + 8) >>> 4))
 *       GRU : h_new  <= sat8((fc_reg + a_z * a_g + 128) >>> 8), c_new <= h_new <<< 4
 *   - cell_step == 2'd2 (ST_CELL_TANH):
 *       LSTM: h_new  <= sat8((a_o * tc + 128) >>> 8)
 *   - Default (ST_GATE_MAC):
 *       GRU : gru_r_h = sat8((a_r * h_old + 128) >>> 8)
 */
`default_nettype none

module nilm_cell (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               cell_en,
    input  wire        [1:0]  cell_step,  // 0:FC, 1:IG, 2:TANH, 3:IDLE/GATE_MAC
    input  wire               cell_type,  // 0 = LSTM, 1 = GRU
    input  wire        [7:0]  a_i,        // UINT8 Q0.8 (or a_z in GRU mode)
    input  wire        [7:0]  a_f,        // UINT8 Q0.8 (or a_r in GRU mode)
    input  wire signed [7:0]  a_g,        // INT8  Q0.7
    input  wire        [7:0]  a_o,        // UINT8 Q0.8
    input  wire signed [15:0] c_old,      // INT16 Q4.11
    input  wire signed [7:0]  h_old,      // INT8  Q0.7 (used in GRU mode)
    output reg  signed [15:0] c_new,      // INT16 Q4.11
    output reg  signed [7:0]  h_new,      // INT8  Q0.7
    output wire signed [7:0]  gru_r_h,    // sat8(rshift_round(a_r * h_old, 8)) for GRU gate 2
    output reg                cell_sat
);

  // c_q34 = sat8((c_new + 64) >>> 7) for tanh(c_new)
  wire signed [16:0] c_s17  = $signed({c_new[15], c_new});
  wire signed [16:0] c_shr7 = (c_s17 + 17'sd64) >>> 7;
  wire               sat_cq = (c_shr7 > 17'sd127) || (c_shr7 < -17'sd128);
  wire signed [7:0]  c_q34  = (c_shr7 > 17'sd127)  ?  8'sd127 :
                              (c_shr7 < -17'sd128) ? -8'sd128 :
                              $signed(c_shr7[7:0]);

  wire [7:0]        unused_sig;
  wire signed [7:0] tc_val;
  nilm_pwl u_cell_pwl (
      .pre_in   (c_q34),
      .sigma_out(unused_sig),
      .tanh_out (tc_val)
  );
  wire _unused_pwl = &{unused_sig, 1'b0};

  // Single shared 10x16 signed multiplier
  wire signed [9:0] a_i_s10   = $signed({2'b00, a_i});
  wire signed [9:0] a_f_s10   = $signed({2'b00, a_f});
  wire signed [9:0] a_o_s10   = $signed({2'b00, a_o});
  wire signed [9:0] one_m_z   = 10'sd256 - a_i_s10;
  wire signed [15:0] h_old_s16 = $signed({{8{h_old[7]}}, h_old});
  wire signed [15:0] a_g_s16   = $signed({{8{a_g[7]}}, a_g});
  wire signed [15:0] tc_s16    = $signed({{8{tc_val[7]}}, tc_val});

  reg signed [9:0]  mul_a;
  reg signed [15:0] mul_b;
  always @(*) begin
    case (cell_step)
      2'd0: begin // ST_CELL_FC
        mul_a = cell_type ? one_m_z : a_f_s10;
        mul_b = cell_type ? h_old_s16 : c_old;
      end
      2'd1: begin // ST_CELL_IG
        mul_a = a_i_s10;
        mul_b = a_g_s16;
      end
      2'd2: begin // ST_CELL_TANH
        mul_a = a_o_s10;
        mul_b = tc_s16;
      end
      default: begin // ST_GATE_MAC (GRU r * h_old)
        mul_a = a_f_s10;
        mul_b = h_old_s16;
      end
    endcase
  end

  wire signed [25:0] prod = mul_a * mul_b;

  // Combinational GRU r * h_old output (used when cell_step == 3)
  wire signed [25:0] rh_rnd = (prod + 26'sd128) >>> 8;
  assign gru_r_h = (rh_rnd > 26'sd127)  ?  8'sd127 :
                   (rh_rnd < -26'sd128) ? -8'sd128 :
                   $signed(rh_rnd[7:0]);

  reg signed [17:0] fc_reg;

  // Step 0 (FC): LSTM (prod + 128) >>> 8; GRU prod
  wire signed [25:0] fc_lstm_w = (prod + 26'sd128) >>> 8;
  wire signed [17:0] fc_next   = cell_type ? $signed(prod[17:0]) : $signed(fc_lstm_w[17:0]);

  // Step 1 (IG):
  // LSTM: c_sum = fc_reg + ((prod + 8) >>> 4)
  wire signed [25:0] ig_rnd    = (prod + 26'sd8) >>> 4;
  wire signed [18:0] c_sum     = $signed({fc_reg[17], fc_reg}) + $signed(ig_rnd[18:0]);
  wire               sat_c     = (c_sum > 19'sd32767) || (c_sum < -19'sd32768);
  wire signed [15:0] c_lstm    = (c_sum > 19'sd32767)  ?  16'sd32767 :
                                 (c_sum < -19'sd32768) ? -16'sd32768 :
                                 $signed(c_sum[15:0]);

  // GRU: gru_sum = (fc_reg + prod + 128) >>> 8
  wire signed [18:0] gru_sum   = $signed({fc_reg[17], fc_reg}) + $signed(prod[18:0]);
  wire signed [18:0] gru_rnd   = (gru_sum + 19'sd128) >>> 8;
  wire               sat_gru   = (gru_rnd > 19'sd127) || (gru_rnd < -19'sd128);
  wire signed [7:0]  h_gru     = (gru_rnd > 19'sd127)  ?  8'sd127 :
                                 (gru_rnd < -19'sd128) ? -8'sd128 :
                                 $signed(gru_rnd[7:0]);
  wire signed [15:0] c_gru     = $signed({{4{h_gru[7]}}, h_gru, 4'b0000});

  // Step 2 (TANH): LSTM h_new = sat8((prod + 128) >>> 8)
  wire signed [25:0] ot_rnd    = (prod + 26'sd128) >>> 8;
  wire               sat_h     = (ot_rnd > 26'sd127) || (ot_rnd < -26'sd128);
  wire signed [7:0]  h_lstm    = (ot_rnd > 26'sd127)  ?  8'sd127 :
                                 (ot_rnd < -26'sd128) ? -8'sd128 :
                                 $signed(ot_rnd[7:0]);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fc_reg   <= 18'sd0;
      c_new    <= 16'sd0;
      h_new    <= 8'sd0;
      cell_sat <= 1'b0;
    end else begin
      cell_sat <= 1'b0;
      if (cell_en) begin
        case (cell_step)
          2'd0: begin
            fc_reg <= fc_next;
          end
          2'd1: begin
            if (cell_type) begin
              c_new    <= c_gru;
              h_new    <= h_gru;
              cell_sat <= sat_gru;
            end else begin
              c_new    <= c_lstm;
              cell_sat <= sat_c;
            end
          end
          2'd2: begin
            if (!cell_type) begin
              h_new    <= h_lstm;
              cell_sat <= sat_cq | sat_h;
            end
          end
          default: ;
        endcase
      end
    end
  end

  wire _unused_bits = &{fc_lstm_w[25:18], ig_rnd[25:19], 1'b0};

endmodule
