/*
 * On-Chip Vector & Context Memory (Default DFF Storage, Sec. 8.5 & Sec. 9 H=16 configuration)
 * - u_mem[0..31]   : 32 x 8-bit feature vector x[0..F-1] (F <= 32)
 * - u_mem[32..47]  : 16 x 8-bit active hidden state h[0..H-1] (H <= 16)
 * - hb_mem[0..15]  : 16 x 8-bit ping-pong hidden state & dense ReLU activation bank (NZ <= 16)
 * - c_mem[0..15]   : 16 x 16-bit LSTM cell state (H <= 16)
 * - deb_mem[0..15] : 16 x 4-bit head debounce shift histories (NHEAD <= 16)
 * - logit_mem[0..7]: 8 x 16-bit last evaluated head logits for RD_LOGIT
 */
`default_nettype none

module nilm_vram (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               ctx_clear,

    // u_mem port
    input  wire               u_we,
    input  wire        [5:0]  u_waddr,
    input  wire signed [7:0]  u_wdata,
    input  wire        [5:0]  u_raddr,
    output wire signed [7:0]  u_rdata,

    // hb_mem port
    input  wire               hb_we,
    input  wire        [4:0]  hb_waddr,
    input  wire signed [7:0]  hb_wdata,
    input  wire        [4:0]  hb_raddr,
    output wire signed [7:0]  hb_rdata,

    // Commit hb_mem[0..15] -> u_mem[32..47] in 1 cycle at H_COMMIT
    input  wire               h_commit,

    // c_mem port
    input  wire               c_we,
    input  wire        [4:0]  c_waddr,
    input  wire signed [15:0] c_wdata,
    input  wire        [4:0]  c_raddr,
    output wire signed [15:0] c_rdata,

    // deb_mem port (16 x 4b) + packed byte read for context save/read
    input  wire               deb_we,
    input  wire        [3:0]  deb_waddr,
    input  wire        [3:0]  deb_wdata,
    input  wire        [3:0]  deb_raddr,
    output wire        [3:0]  deb_rdata,
    input  wire        [1:0]  deb_pair_idx,
    output wire        [7:0]  deb_pair_rdata,

    // logit_mem port
    input  wire               logit_we,
    input  wire        [3:0]  logit_waddr,
    input  wire signed [15:0] logit_wdata,
    input  wire        [3:0]  logit_raddr,
    output wire signed [15:0] logit_rdata
);

  reg signed [7:0]  u_mem     [0:47];
  reg signed [7:0]  hb_mem    [0:15];
  reg signed [15:0] c_mem     [0:15];
  reg        [3:0]  deb_mem   [0:15];
  reg signed [15:0] logit_mem [0:7];

  assign u_rdata        = (u_raddr < 6'd48) ? u_mem[u_raddr] : 8'sd0;
  assign hb_rdata       = (hb_raddr < 5'd16) ? hb_mem[hb_raddr[3:0]] : 8'sd0;
  assign c_rdata        = (c_raddr < 5'd16) ? c_mem[c_raddr[3:0]] : 16'sd0;
  assign deb_rdata      = deb_mem[deb_raddr];
  assign deb_pair_rdata = {deb_mem[{1'b0, deb_pair_idx, 1'b1}], deb_mem[{1'b0, deb_pair_idx, 1'b0}]};
  assign logit_rdata    = logit_mem[logit_raddr[2:0]];

  integer i;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (i = 0; i < 48; i = i + 1) u_mem[i] <= 8'sd0;
      for (i = 0; i < 16; i = i + 1) begin
        hb_mem[i]  <= 8'sd0;
        c_mem[i]   <= 16'sd0;
        deb_mem[i] <= 4'd0;
      end
      for (i = 0; i < 8; i = i + 1) begin
        logit_mem[i] <= 16'sd0;
      end
    end else begin
      if (ctx_clear) begin
        for (i = 32; i < 48; i = i + 1) u_mem[i] <= 8'sd0;
        for (i = 0; i < 16; i = i + 1) begin
          hb_mem[i]  <= 8'sd0;
          c_mem[i]   <= 16'sd0;
          deb_mem[i] <= 4'd0;
        end
        for (i = 0; i < 8; i = i + 1) begin
          logit_mem[i] <= 16'sd0;
        end
      end else begin
        if (h_commit) begin
          for (i = 0; i < 16; i = i + 1) begin
            u_mem[32 + i] <= hb_mem[i];
          end
        end else if (u_we && (u_waddr < 6'd48)) begin
          u_mem[u_waddr] <= u_wdata;
        end

        if (hb_we && (hb_waddr < 5'd16)) begin
          hb_mem[hb_waddr[3:0]] <= hb_wdata;
        end

        if (c_we && (c_waddr < 5'd16)) begin
          c_mem[c_waddr[3:0]] <= c_wdata;
        end

        if (deb_we) begin
          deb_mem[deb_waddr] <= deb_wdata;
        end

        if (logit_we && !logit_waddr[3]) begin
          logit_mem[logit_waddr[2:0]] <= logit_wdata;
        end
      end
    end
  end

  wire _unused = &{logit_raddr[3], 1'b0};

endmodule
