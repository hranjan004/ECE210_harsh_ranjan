/*
 * QSPI Flash & PSRAM Controller (§8.2)
 * - Pinout (uio[7:0]):
 *     [0] CS0_N (Flash)
 *     [1] SD0   [2] SD1   [3] SCK   [4] SD2   [5] SD3
 *     [6] CS1_N (PSRAM A) [7] CS2_N (PSRAM B)
 * - Unified 1-4-4 EBh Quad Read (20 SCK preamble: 8 cmd + 6 addr + 2 mode Fh + 4 dummy)
 * - PSRAM 1-4-4 38h Quad Write (14 SCK preamble: 8 cmd + 6 addr + 0 wait)
 * - 66h / 99h Reset sequence for Flash & PSRAM
 * - Programmable SCK divider (/2, /4, /8, /16), RD_LAT (0..3 clk), RD_HALF
 * - Automatic PSRAM burst chunking at psram_burst_max to enforce tCEM <= 8 us
 * - Safe Bus Handover (uio_oe == 8'h00 in reset, before init, and during BUS_GNT)
 */
`default_nettype none

module nilm_qspi (
    input  wire        clk,
    input  wire        rst_n,

    // Configuration
    input  wire [1:0]  sck_div_sel,      // 0:/2, 1:/4, 2:/8, 3:/16
    input  wire [1:0]  rd_lat,           // 0..3 extra clk cycles on read nibble capture
    input  wire        rd_half,          // 0: capture on SCK fall (1/2 period after launch), 1: +half SCK
    input  wire [7:0]  psram_burst_max,  // Max bytes per PSRAM CE# low burst (default 64)

    // Bus handover control
    input  wire        bus_req,          // External BUS_REQ pin or BUS_RELEASE command
    input  wire        bus_acquire_pulse,// Acquire bus (QSPI_INIT or BUS_ACQUIRE)
    input  wire        seq_idle,         // Sequencer is idle (safe to grant bus)
    output reg         bus_gnt,          // 1 when host owns QSPI bus (uio_oe == 0)
    output reg         bus_owned,        // 1 when chip drives QSPI pins

    // Transaction interface from Sequencer
    input  wire        cmd_valid,
    input  wire [1:0]  cmd_type,         // 1: INIT_66_99, 2: QUAD_READ (EBh), 3: QUAD_WRITE (38h)
    input  wire        cmd_dev,          // 0: Flash (CS0), 1: PSRAM A (CS1)
    input  wire [23:0] cmd_addr,
    input  wire [15:0] cmd_len,          // Number of bytes to read or write
    input  wire        cmd_abort,        // Abort active transfer and deassert CS
    output wire        cmd_busy,
    output reg         cmd_done,

    // Streaming Read Byte output (Flash supports stall via rd_ready; PSRAM never stalls)
    output reg         rd_valid,
    output reg  [7:0]  rd_data,
    input  wire        rd_ready,

    // Streaming Write Byte input (PSRAM 38h)
    output wire        wr_req,           // Asserts 1 cycle to request next write byte
    input  wire [7:0]  wr_data,

    // Tiny Tapeout bidirectional I/O pads
    input  wire [7:0]  uio_in,
    output wire [7:0]  uio_out,
    output wire [7:0]  uio_oe
);

  // FSM states
  localparam [3:0]
      Q_IDLE      = 4'd0,
      Q_RST_CMD   = 4'd1,
      Q_RST_GAP   = 4'd2,
      Q_PRE_CMD   = 4'd3,
      Q_PRE_ADDR  = 4'd4,
      Q_PRE_MODE  = 4'd5,
      Q_PRE_DUMMY = 4'd6,
      Q_RD_DATA   = 4'd7,
      Q_RD_TAIL   = 4'd8,
      Q_WR_DATA   = 4'd9,
      Q_CS_GAP    = 4'd10,
      Q_HANDOVER  = 4'd11;

  reg [3:0]  state;
  reg        active_dev;     // 0: Flash, 1: PSRAM A
  reg [1:0]  active_type;
  reg [23:0] cur_addr;
  reg [15:0] bytes_rem;
  reg [7:0]  burst_cnt;
  reg [4:0]  step_cnt;
  reg [2:0]  init_phase;     // 0:Flash 66h, 1:Flash 99h, 2:PSRAM 66h, 3:PSRAM 99h

  // SCK divider: half_period in clk cycles = 1 << sck_div_sel (1, 2, 4, 8 clk per half SCK)
  wire [3:0] half_period = (sck_div_sel == 2'd0) ? 4'd1 :
                           (sck_div_sel == 2'd1) ? 4'd2 :
                           (sck_div_sel == 2'd2) ? 4'd4 : 4'd8;
  reg  [3:0] div_cnt;
  reg        sck_reg;

  // Pad outputs
  reg        cs0_n;
  reg        cs1_n;
  reg        cs2_n;
  reg        sd_oe;
  reg [3:0]  sd_out;

  // Delayed read nibble capture pipeline for RD_LAT (0..3) and RD_HALF
  wire [3:0] sd_in_nib = {uio_in[5], uio_in[4], uio_in[2], uio_in[1]};
  reg  [3:0] nib_pipe [0:4];
  reg  [5:0] cap_hi_pipe;
  reg  [5:0] cap_lo_pipe;
  reg  [3:0] rd_hi_nib;

  wire [7:0] eff_burst_max = (psram_burst_max == 8'd0) ? 8'd64 : psram_burst_max;

  // Stall condition during Flash read when consumer hasn't consumed rd_valid yet
  wire flash_stall = (state == Q_RD_DATA) && (!active_dev) && rd_valid && (!rd_ready);

  wire tick_en  = (state != Q_IDLE) && (!flash_stall);
  wire half_tick = tick_en && (div_cnt == (half_period - 4'd1));
  wire sck_rise  = half_tick && (!sck_reg);
  wire sck_fall  = half_tick && sck_reg;

  // Request next write byte right as preamble finishes or after low nibble launches
  assign wr_req = ((state == Q_PRE_ADDR) && (step_cnt == 5'd5) && sck_fall && (active_type == 2'd3)) ||
                  ((state == Q_WR_DATA) && (step_cnt[0] == 1'b1) && sck_fall &&
                   (bytes_rem > 16'd1) && ((burst_cnt + 8'd1) < eff_burst_max));

  reg [7:0] wr_byte_lat;

  // Capture trigger for read nibbles:
  // By default (rd_half=0), the slave drives SD on falling SCK and we sample on the next falling SCK
  // (one full SCK after launch, or half SCK after rising edge), shifted by rd_lat clk cycles.
  wire raw_cap_trig = (state == Q_RD_DATA) && (rd_half ? sck_fall : sck_rise);
  wire raw_cap_hi   = raw_cap_trig && (step_cnt[0] == 1'b0);
  wire raw_cap_lo   = raw_cap_trig && (step_cnt[0] == 1'b1);

  wire [2:0] lat_idx    = {1'b0, rd_lat} + (rd_half ? 3'd1 : 3'd0);
  wire       cap_hi_now = (lat_idx == 3'd0) ? raw_cap_hi : cap_hi_pipe[lat_idx - 3'd1];
  wire       cap_lo_now = (lat_idx == 3'd0) ? raw_cap_lo : cap_lo_pipe[lat_idx - 3'd1];

  integer k;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (k = 0; k < 5; k = k + 1) nib_pipe[k] <= 4'h0;
      cap_hi_pipe <= 6'd0;
      cap_lo_pipe <= 6'd0;
      rd_hi_nib   <= 4'h0;
      rd_valid    <= 1'b0;
      rd_data     <= 8'h00;
    end else begin
      nib_pipe[0] <= sd_in_nib;
      for (k = 1; k < 5; k = k + 1) nib_pipe[k] <= nib_pipe[k - 1];
      cap_hi_pipe <= {cap_hi_pipe[4:0], raw_cap_hi};
      cap_lo_pipe <= {cap_lo_pipe[4:0], raw_cap_lo};

      if (cmd_abort || (cmd_valid && state == Q_IDLE)) begin
        rd_valid <= 1'b0;
      end else if (rd_valid && rd_ready) begin
        rd_valid <= 1'b0;
      end

      if (cap_hi_now) begin
        rd_hi_nib <= sd_in_nib;
      end
      if (cap_lo_now) begin
        rd_data  <= {rd_hi_nib, sd_in_nib};
        rd_valid <= 1'b1;
      end
    end
  end

  // Main QSPI protocol & bus handover FSM
  wire [7:0] rst_cmd_byte    = init_phase[0] ? 8'h99 : 8'h66;
  wire [7:0] pre_cmd_byte    = (active_type == 2'd3) ? 8'h38 : 8'hEB;
  wire [2:0] next_init_phase = init_phase + 3'd1;

  // Select address nibble (step_cnt 0..5 -> nibbles 5..0)
  reg [3:0] addr_nib;
  always @(*) begin
    case (step_cnt[2:0])
      3'd0: addr_nib = cur_addr[23:20];
      3'd1: addr_nib = cur_addr[19:16];
      3'd2: addr_nib = cur_addr[15:12];
      3'd3: addr_nib = cur_addr[11:8];
      3'd4: addr_nib = cur_addr[7:4];
      default: addr_nib = cur_addr[3:0];
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state       <= Q_IDLE;
      bus_gnt     <= 1'b0;
      bus_owned   <= 1'b0;
      active_dev  <= 1'b0;
      active_type <= 2'd0;
      cur_addr    <= 24'd0;
      bytes_rem   <= 16'd0;
      burst_cnt   <= 8'd0;
      step_cnt    <= 5'd0;
      init_phase  <= 3'd0;
      div_cnt     <= 4'd0;
      sck_reg     <= 1'b0;
      cs0_n       <= 1'b1;
      cs1_n       <= 1'b1;
      cs2_n       <= 1'b1;
      sd_oe       <= 1'b0;
      sd_out      <= 4'h0;
      wr_byte_lat <= 8'h00;
      cmd_done    <= 1'b0;
    end else begin
      cmd_done <= 1'b0;

      if (wr_req) begin
        wr_byte_lat <= wr_data;
      end

      if (cmd_abort) begin
        state   <= Q_IDLE;
        sck_reg <= 1'b0;
        cs0_n   <= 1'b1;
        cs1_n   <= 1'b1;
        cs2_n   <= 1'b1;
        sd_oe   <= 1'b0;
        div_cnt <= 4'd0;
      end else begin
        // Clock divider
        if (!tick_en) begin
          div_cnt <= 4'd0;
        end else if (half_tick) begin
          div_cnt <= 4'd0;
          if (state != Q_RST_GAP && state != Q_CS_GAP && state != Q_HANDOVER && state != Q_RD_TAIL) begin
            sck_reg <= ~sck_reg;
          end else begin
            sck_reg <= 1'b0;
          end
        end else begin
          div_cnt <= div_cnt + 4'd1;
        end

        case (state)
          Q_IDLE: begin
            sck_reg <= 1'b0;
            cs0_n   <= 1'b1;
            cs1_n   <= 1'b1;
            cs2_n   <= 1'b1;
            sd_oe   <= 1'b0;
            if (bus_acquire_pulse) begin
              bus_owned <= 1'b1;
              bus_gnt   <= 1'b0;
            end
            if (bus_req && seq_idle && !cmd_valid) begin
              if (!bus_gnt) begin
                state    <= Q_HANDOVER;
                step_cnt <= 5'd0;
              end
            end else if (!bus_req && bus_gnt) begin
              bus_gnt <= 1'b0;
            end else if (cmd_valid && !bus_gnt) begin
              bus_owned   <= 1'b1;
              active_type <= cmd_type;
              active_dev  <= cmd_dev;
              cur_addr    <= cmd_addr;
              bytes_rem   <= cmd_len;
              burst_cnt   <= 8'd0;
              step_cnt    <= 5'd0;
              if (cmd_type == 2'd1) begin
                // INIT 66h/99h on Flash then PSRAM
                init_phase <= 3'd0;
                cs0_n      <= 1'b0;
                cs1_n      <= 1'b1;
                sd_oe      <= 1'b1;
                sd_out     <= {2'b11, 1'b0, 1'b0}; // 66h MSB = 0
                state      <= Q_RST_CMD;
              end else begin
                // EBh Read or 38h Write
                cs0_n  <= (cmd_dev == 1'b0) ? 1'b0 : 1'b1;
                cs1_n  <= (cmd_dev == 1'b1) ? 1'b0 : 1'b1;
                sd_oe  <= 1'b1;
                sd_out <= {2'b11, 1'b0, (cmd_type == 2'd3) ? 1'b0 : 1'b1}; // MSB of 38h(0) or EBh(1)
                state  <= Q_PRE_CMD;
              end
            end
          end

          Q_HANDOVER: begin
            cs0_n <= 1'b1;
            cs1_n <= 1'b1;
            cs2_n <= 1'b1;
            sd_oe <= 1'b0;
            if (half_tick) begin
              if (step_cnt == 5'd3) begin
                bus_gnt   <= 1'b1;
                bus_owned <= 1'b0;
                state     <= Q_IDLE;
              end else begin
                step_cnt <= step_cnt + 5'd1;
              end
            end
          end

          Q_RST_CMD: begin
            if (sck_fall) begin
              if (step_cnt == 5'd7) begin
                cs0_n    <= 1'b1;
                cs1_n    <= 1'b1;
                sd_oe    <= 1'b0;
                step_cnt <= 5'd0;
                state    <= Q_RST_GAP;
              end else begin
                step_cnt <= step_cnt + 5'd1;
                sd_out   <= {2'b11, 1'b0, rst_cmd_byte[6 - step_cnt[2:0]]};
              end
            end
          end

          Q_RST_GAP: begin
            if (half_tick) begin
              if (step_cnt == 5'd3) begin
                step_cnt <= 5'd0;
                if (init_phase == 3'd3) begin
                  cmd_done <= 1'b1;
                  state    <= Q_IDLE;
                end else begin
                  init_phase <= next_init_phase;
                  cs0_n      <= (next_init_phase < 3'd2) ? 1'b0 : 1'b1;
                  cs1_n      <= (next_init_phase >= 3'd2) ? 1'b0 : 1'b1;
                  sd_oe      <= 1'b1;
                  // Both 66h and 99h have bit7 = (99h -> 1, 66h -> 0)
                  sd_out     <= {2'b11, 1'b0, next_init_phase[0]};
                  state      <= Q_RST_CMD;
                end
              end else begin
                step_cnt <= step_cnt + 5'd1;
              end
            end
          end

          Q_PRE_CMD: begin
            if (sck_fall) begin
              if (step_cnt == 5'd7) begin
                step_cnt <= 5'd0;
                sd_oe    <= 1'b1;
                sd_out   <= cur_addr[23:20];
                state    <= Q_PRE_ADDR;
              end else begin
                step_cnt <= step_cnt + 5'd1;
                sd_out   <= {2'b11, 1'b0, pre_cmd_byte[6 - step_cnt[2:0]]};
              end
            end
          end

          Q_PRE_ADDR: begin
            if (sck_fall) begin
              if (step_cnt == 5'd5) begin
                step_cnt <= 5'd0;
                if (active_type == 2'd3) begin
                  // PSRAM 38h write has 0 wait cycles: go straight to Q_WR_DATA
                  sd_oe  <= 1'b1;
                  sd_out <= wr_data[7:4];
                  state  <= Q_WR_DATA;
                end else begin
                  // EBh read: 2 SCK mode (Fh)
                  sd_oe  <= 1'b1;
                  sd_out <= 4'hF;
                  state  <= Q_PRE_MODE;
                end
              end else begin
                step_cnt <= step_cnt + 5'd1;
                case (step_cnt[2:0])
                  3'd0: sd_out <= cur_addr[19:16];
                  3'd1: sd_out <= cur_addr[15:12];
                  3'd2: sd_out <= cur_addr[11:8];
                  3'd3: sd_out <= cur_addr[7:4];
                  default: sd_out <= cur_addr[3:0];
                endcase
              end
            end
          end

          Q_PRE_MODE: begin
            if (sck_fall) begin
              if (step_cnt == 5'd1) begin
                step_cnt <= 5'd0;
                sd_oe    <= 1'b0; // Release SD bus for 4 dummy SCKs + read phase!
                sd_out   <= 4'h0;
                state    <= Q_PRE_DUMMY;
              end else begin
                step_cnt <= step_cnt + 5'd1;
                sd_out   <= 4'hF;
              end
            end
          end

          Q_PRE_DUMMY: begin
            sd_oe <= 1'b0;
            if (sck_fall) begin
              if (step_cnt == 5'd3) begin
                step_cnt <= 5'd0;
                state    <= Q_RD_DATA;
              end else begin
                step_cnt <= step_cnt + 5'd1;
              end
            end
          end

          Q_RD_DATA: begin
            sd_oe <= 1'b0;
            if (sck_fall) begin
              if (step_cnt[0] == 1'b0) begin
                step_cnt <= 5'd1;
              end else begin
                step_cnt  <= 5'd0;
                bytes_rem <= bytes_rem - 16'd1;
                burst_cnt <= burst_cnt + 8'd1;
                cur_addr  <= cur_addr + 24'd1;
                if (bytes_rem == 16'd1) begin
                  cs0_n    <= 1'b1;
                  cs1_n    <= 1'b1;
                  step_cnt <= 5'd0;
                  state    <= Q_RD_TAIL;
                end else if (active_dev && ((burst_cnt + 8'd1) >= eff_burst_max)) begin
                  // Split PSRAM read burst to obey tCEM!
                  cs1_n     <= 1'b1;
                  burst_cnt <= 8'd0;
                  step_cnt  <= 5'd0;
                  state     <= Q_CS_GAP;
                end
              end
            end
          end

          Q_RD_TAIL: begin
            // Wait for the delayed read capture pipeline to flush the final byte
            cs0_n <= 1'b1;
            cs1_n <= 1'b1;
            sd_oe <= 1'b0;
            if (step_cnt >= 5'd6 && !rd_valid) begin
              cmd_done <= 1'b1;
              state    <= Q_IDLE;
            end else if (step_cnt < 5'd7) begin
              step_cnt <= step_cnt + 5'd1;
            end
          end

          Q_WR_DATA: begin
            sd_oe <= 1'b1;
            if (sck_fall) begin
              if (step_cnt[0] == 1'b0) begin
                step_cnt <= 5'd1;
                sd_out   <= wr_byte_lat[3:0];
              end else begin
                step_cnt  <= 5'd0;
                bytes_rem <= bytes_rem - 16'd1;
                burst_cnt <= burst_cnt + 8'd1;
                cur_addr  <= cur_addr + 24'd1;
                if (bytes_rem == 16'd1) begin
                  cs0_n    <= 1'b1;
                  cs1_n    <= 1'b1;
                  sd_oe    <= 1'b0;
                  cmd_done <= 1'b1;
                  state    <= Q_IDLE;
                end else if (active_dev && ((burst_cnt + 8'd1) >= eff_burst_max)) begin
                  cs1_n     <= 1'b1;
                  sd_oe     <= 1'b0;
                  burst_cnt <= 8'd0;
                  step_cnt  <= 5'd0;
                  state     <= Q_CS_GAP;
                end else begin
                  sd_out <= wr_data[7:4];
                end
              end
            end
          end

          Q_CS_GAP: begin
            cs0_n <= 1'b1;
            cs1_n <= 1'b1;
            sd_oe <= 1'b0;
            if (half_tick) begin
              if (step_cnt == 5'd5) begin
                // Resume next PSRAM chunk
                step_cnt <= 5'd0;
                cs1_n    <= 1'b0;
                sd_oe    <= 1'b1;
                sd_out   <= {2'b11, 1'b0, (active_type == 2'd3) ? 1'b0 : 1'b1};
                state    <= Q_PRE_CMD;
              end else begin
                step_cnt <= step_cnt + 5'd1;
              end
            end
          end

          default: state <= Q_IDLE;
        endcase
      end
    end
  end

  assign cmd_busy = (state != Q_IDLE);

  // Pin mapping (§4, §7.1):
  // uio[0]=CS0, [1]=SD0, [2]=SD1, [3]=SCK, [4]=SD2, [5]=SD3, [6]=CS1, [7]=CS2
  assign uio_out = {cs2_n, cs1_n, sd_out[3], sd_out[2], sck_reg, sd_out[1], sd_out[0], cs0_n};
  assign uio_oe  = (!rst_n || bus_gnt || !bus_owned) ? 8'h00 :
                   {2'b11, sd_oe ? 2'b11 : 2'b00, 1'b1, sd_oe ? 2'b11 : 2'b00, 1'b1};

endmodule
