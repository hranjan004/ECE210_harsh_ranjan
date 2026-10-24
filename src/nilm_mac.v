/*
 * Signed INT8 MAC & Power-of-Two Round-Half-Up Requantizer (Sec. 8.3)
 * - 8x8 -> 16 signed multiplier
 * - 24-bit saturating accumulator with pre-shifted INT16 bias load
 * - Round-half-up shift (s in 0..15) with sat8 and sat16 outputs
 */
`default_nettype none

module nilm_mac (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clr_sat,       // Clear sticky sat flag
    input  wire               load_bias,     // Load bias_in <<< bias_shift into acc
    input  wire               mac_en,        // Accumulate w_in * u_in into acc
    input  wire signed [15:0] bias_in,
    input  wire        [3:0]  bias_shift,
    input  wire signed [7:0]  w_in,
    input  wire signed [7:0]  u_in,
    input  wire        [3:0]  req_shift,
    output reg  signed [23:0] acc_out,
    output wire signed [7:0]  req8_out,
    output wire               req8_sat,
    output wire signed [15:0] req16_out,
    output wire               req16_sat,
    output reg                sat_seen
);

  // Bias shifted left by bias_shift (up to 15 bits -> 31-bit signed intermediate)
  wire signed [31:0] bias_ext_in = $signed({{16{bias_in[15]}}, bias_in});
  wire signed [31:0] bias_ext    = bias_ext_in <<< bias_shift;
  wire               bias_ovf    = (bias_ext > 32'sd8388607) || (bias_ext < -32'sd8388608);
  wire signed [23:0] bias_sat    = (bias_ext > 32'sd8388607)  ?  24'sd8388607 :
                                   (bias_ext < -32'sd8388608) ? -24'sd8388608 :
                                   $signed(bias_ext[23:0]);

  // Signed 8x8 -> 16 product and 25-bit signed sum
  wire signed [15:0] prod     = w_in * u_in;
  wire signed [24:0] acc_s25  = $signed({acc_out[23], acc_out});
  wire signed [24:0] prod_s25 = $signed({{9{prod[15]}}, prod});
  wire signed [24:0] sum_ext  = acc_s25 + prod_s25;
  wire               mac_ovf  = (sum_ext > 25'sd8388607) || (sum_ext < -25'sd8388608);
  wire signed [23:0] mac_sat  = (sum_ext > 25'sd8388607)  ?  24'sd8388607 :
                                (sum_ext < -25'sd8388608) ? -24'sd8388608 :
                                $signed(sum_ext[23:0]);

  // Round-half-up shift: (acc_out + (1 << (s-1))) >>> s
  wire [23:0]        round_mag   = (req_shift == 4'd0) ? 24'd0 : (24'd1 << (req_shift - 4'd1));
  wire signed [24:0] round_add   = $signed({1'b0, round_mag});
  wire signed [24:0] acc_rounded = acc_s25 + round_add;
  wire signed [24:0] acc_shifted = (req_shift == 4'd0) ? acc_s25 : (acc_rounded >>> req_shift);

  assign req8_sat  = (acc_shifted > 25'sd127) || (acc_shifted < -25'sd128);
  assign req8_out  = (acc_shifted > 25'sd127)  ?  8'sd127 :
                     (acc_shifted < -25'sd128) ? -8'sd128 :
                     $signed(acc_shifted[7:0]);

  assign req16_sat = (acc_shifted > 25'sd32767) || (acc_shifted < -25'sd32768);
  assign req16_out = (acc_shifted > 25'sd32767)  ?  16'sd32767 :
                     (acc_shifted < -25'sd32768) ? -16'sd32768 :
                     $signed(acc_shifted[15:0]);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      acc_out  <= 24'sd0;
      sat_seen <= 1'b0;
    end else begin
      if (clr_sat) begin
        sat_seen <= 1'b0;
      end
      if (load_bias) begin
        acc_out <= bias_sat;
        if (bias_ovf) sat_seen <= 1'b1;
      end else if (mac_en) begin
        acc_out <= mac_sat;
        if (mac_ovf) sat_seen <= 1'b1;
      end
    end
  end

endmodule
