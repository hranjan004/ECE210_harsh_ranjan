/*
 * PLAN-style Piecewise-Linear (PWL) Sigmoid and Tanh (§8.4)
 * Shared combinational datapath:
 *   sigma_pwl(pre_q34) -> UINT8 Q0.8 [0..255]
 *   tanh_pwl(pre_q34)  -> INT8  Q0.7 [-127..127] via 2*sigma(2x) - 1
 */
`default_nettype none

module nilm_pwl (
    input  wire signed [7:0] pre_in,     // INT8 Q3.4 [-128..127]
    output wire        [7:0] sigma_out,  // UINT8 Q0.8 [0..255]
    output wire signed [7:0] tanh_out    // INT8 Q0.7 [-127..127]
);

  wire is_neg = pre_in[7];
  wire [8:0] abs_x = is_neg ? (9'd0 - {pre_in[7], pre_in}) : {1'b0, pre_in};
  wire [9:0] abs_2x = {abs_x, 1'b0};

  // Helper function for _pwl_pos_q08(ax) in [128..256]
  function [8:0] pwl_pos_q08;
    input [9:0] ax;
    begin
      if (ax >= 10'd80)
        pwl_pos_q08 = 9'd256;
      else if (ax >= 10'd38)
        pwl_pos_q08 = (ax[8:1]) + 9'd216;
      else if (ax >= 10'd16)
        pwl_pos_q08 = {ax[6:0], 1'b0} + 9'd160;
      else
        pwl_pos_q08 = {ax[5:0], 2'b00} + 9'd128;
    end
  endfunction

  wire [8:0] sig_pos = pwl_pos_q08({1'b0, abs_x});
  wire [8:0] sig_neg = 9'd256 - sig_pos;
  assign sigma_out = is_neg ? sig_neg[7:0] : (sig_pos > 9'd255 ? 8'd255 : sig_pos[7:0]);

  wire [8:0] tanh_pos_q08 = pwl_pos_q08(abs_2x);
  wire [8:0] tanh_mag_raw = tanh_pos_q08 - 9'd128;
  wire [7:0] tanh_mag     = (tanh_mag_raw > 9'd127) ? 8'd127 : tanh_mag_raw[7:0];
  wire [7:0] tanh_neg_val = 8'd0 - tanh_mag;

  assign tanh_out = (pre_in == 8'sd0) ? 8'sd0 :
                    (is_neg ? $signed(tanh_neg_val) : $signed(tanh_mag));

endmodule
