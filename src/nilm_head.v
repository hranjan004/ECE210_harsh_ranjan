/*
 * Linear Logit Head Comparator & N-of-M Debounce Unit (Sec. 8.7)
 */
`default_nettype none

module nilm_head (
    input  wire signed [15:0] logit_in,
    input  wire signed [15:0] theta_in,
    input  wire        [3:0]  deb_hist_in,
    input  wire               deb_en,
    input  wire        [3:0]  deb_n,
    input  wire        [3:0]  deb_m,
    output wire               raw_on,
    output wire        [3:0]  deb_hist_out,
    output wire               head_on
);

  assign raw_on       = (logit_in >= theta_in);
  assign deb_hist_out = {deb_hist_in[2:0], raw_on};

  wire [3:0] win_mask = (deb_m <= 4'd1) ? 4'b0001 :
                        (deb_m == 4'd2) ? 4'b0011 :
                        (deb_m == 4'd3) ? 4'b0111 :
                                          4'b1111;

  wire [3:0] masked   = deb_hist_out & win_mask;
  wire [2:0] popcnt   = {2'b00, masked[0]} +
                        {2'b00, masked[1]} +
                        {2'b00, masked[2]} +
                        {2'b00, masked[3]};

  assign head_on = (deb_en && (deb_n != 4'd0)) ? ({1'b0, popcnt} >= deb_n) : raw_on;

  wire _unused = &{deb_hist_in[3], 1'b0};

endmodule
