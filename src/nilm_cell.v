/*
 * LSTM & GRU Cell Elementwise Update Unit (Sec. 8.4)
 * Exact match to golden/nilm_golden.py:
 *   LSTM (cell_type == 0):
 *     fc    = rshift_round(a_f[u8] * c_old[s16], 8)
 *     ig    = rshift_round(a_i[u8] * a_g[s8], 4)
 *     c_new = sat16(fc + ig)
 *     tc    = tanh_pwl(sat8(rshift_round(c_new, 7)))
 *     h_new = sat8(rshift_round(a_o[u8] * tc[s8], 8))
 *   GRU (cell_type == 1):
 *     h_new = sat8(rshift_round((256 - a_z)*h_old + a_z*a_g, 8))
 *     c_new = h_new <<< 4
 */
`default_nettype none

module nilm_cell (
    input  wire               cell_type,  // 0 = LSTM, 1 = GRU
    input  wire        [7:0]  a_i,        // UINT8 Q0.8 (or a_z in GRU mode)
    input  wire        [7:0]  a_f,        // UINT8 Q0.8 (or a_r in GRU mode)
    input  wire signed [7:0]  a_g,        // INT8  Q0.7
    input  wire        [7:0]  a_o,        // UINT8 Q0.8
    input  wire signed [15:0] c_old,      // INT16 Q4.11
    input  wire signed [7:0]  h_old,      // INT8  Q0.7 (used in GRU mode)
    output wire signed [15:0] c_new,      // INT16 Q4.11
    output wire signed [7:0]  h_new,      // INT8  Q0.7
    output wire signed [7:0]  gru_r_h,    // sat8(rshift_round(a_r * h_old, 8)) for GRU gate 2
    output wire               cell_sat
);

  wire signed [8:0] a_i_s = $signed({1'b0, a_i});
  wire signed [8:0] a_f_s = $signed({1'b0, a_f});
  wire signed [8:0] a_o_s = $signed({1'b0, a_o});

  // fc = (a_f * c_old + 128) >>> 8
  wire signed [24:0] prod_fc = a_f_s * c_old;
  wire signed [24:0] fc_rnd  = (prod_fc + 25'sd128) >>> 8;

  // ig = (a_i * a_g + 8) >>> 4
  wire signed [16:0] prod_ig = a_i_s * a_g;
  wire signed [16:0] ig_rnd  = (prod_ig + 17'sd8) >>> 4;
  wire signed [24:0] ig_s25  = $signed({{8{ig_rnd[16]}}, ig_rnd});

  // c_sum = fc + ig -> sat16
  wire signed [24:0] c_sum    = fc_rnd + ig_s25;
  wire               sat_c    = (c_sum > 25'sd32767) || (c_sum < -25'sd32768);
  wire signed [15:0] c_lstm   = (c_sum > 25'sd32767)  ?  16'sd32767 :
                                (c_sum < -25'sd32768) ? -16'sd32768 :
                                $signed(c_sum[15:0]);

  // c_q34 = sat8((c_lstm + 64) >>> 7)
  wire signed [16:0] c_s17    = $signed({c_lstm[15], c_lstm});
  wire signed [16:0] c_shr7   = (c_s17 + 17'sd64) >>> 7;
  wire               sat_cq   = (c_shr7 > 17'sd127) || (c_shr7 < -17'sd128);
  wire signed [7:0]  c_q34    = (c_shr7 > 17'sd127)  ?  8'sd127 :
                                (c_shr7 < -17'sd128) ? -8'sd128 :
                                $signed(c_shr7[7:0]);

  wire [7:0]        unused_sig;
  wire signed [7:0] tc_val;
  nilm_pwl u_cell_pwl (
      .pre_in   (c_q34),
      .sigma_out(unused_sig),
      .tanh_out (tc_val)
  );

  // h_lstm = sat8((a_o * tc_val + 128) >>> 8)
  wire signed [16:0] prod_ot  = a_o_s * tc_val;
  wire signed [16:0] ot_rnd   = (prod_ot + 17'sd128) >>> 8;
  wire               sat_h    = (ot_rnd > 17'sd127) || (ot_rnd < -17'sd128);
  wire signed [7:0]  h_lstm   = (ot_rnd > 17'sd127)  ?  8'sd127 :
                                (ot_rnd < -17'sd128) ? -8'sd128 :
                                $signed(ot_rnd[7:0]);

  // GRU helpers:
  // 1) Gated recurrent operand: sat8((a_r * h_old + 128) >>> 8)
  wire signed [16:0] prod_rh  = a_f_s * h_old;
  wire signed [16:0] rh_rnd   = (prod_rh + 17'sd128) >>> 8;
  assign gru_r_h = (rh_rnd > 17'sd127)  ?  8'sd127 :
                   (rh_rnd < -17'sd128) ? -8'sd128 :
                   $signed(rh_rnd[7:0]);

  // 2) GRU state update: ((256 - a_z)*h_old + a_z*a_g + 128) >>> 8
  wire signed [9:0]  a_z_s10  = $signed({2'b00, a_i});
  wire signed [9:0]  one_m_z  = 10'sd256 - a_z_s10;
  wire signed [17:0] gru_sum  = (one_m_z * h_old) + (a_i_s * a_g);
  wire signed [17:0] gru_rnd  = (gru_sum + 18'sd128) >>> 8;
  wire               sat_gru  = (gru_rnd > 18'sd127) || (gru_rnd < -18'sd128);
  wire signed [7:0]  h_gru    = (gru_rnd > 18'sd127)  ?  8'sd127 :
                                (gru_rnd < -18'sd128) ? -8'sd128 :
                                $signed(gru_rnd[7:0]);
  wire signed [15:0] c_gru    = $signed({{4{h_gru[7]}}, h_gru, 4'b0000});

  assign c_new    = cell_type ? c_gru  : c_lstm;
  assign h_new    = cell_type ? h_gru  : h_lstm;
  assign cell_sat = cell_type ? sat_gru : (sat_c | sat_cq | sat_h);

endmodule
