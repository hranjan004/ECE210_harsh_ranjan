/*
 * Oversampled Host SPI-Slave Interface (§7.1, §7.2, §8.1)
 * - Mode 0 (CPOL=0, CPHA=0), MSB first
 * - 2-FF synchronizers on HOST_CS_N, HOST_SCK, HOST_MOSI (single clock domain N6)
 * - Samples MOSI on synchronized SCK rising edge; shifts MISO on synchronized SCK falling edge
 * - Detects mid-byte CS_N rising edge abort (ERR_FRAME 0x01)
 */
`default_nettype none

module nilm_host_spi (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       spi_cs_n,
    input  wire       spi_sck,
    input  wire       spi_mosi,
    output wire       spi_miso,

    output wire       frame_start,   // 1-clk pulse when CS_N falls
    output wire       frame_end,     // 1-clk pulse when CS_N rises
    output reg        frame_err,     // 1-clk pulse if CS_N rises with bit_cnt != 0
    output reg        rx_valid,      // 1-clk pulse when a full byte is received
    output reg  [7:0] rx_byte,
    input  wire [7:0] tx_byte        // Byte to transmit on next byte boundary
);

  // 2-FF synchronizers (plus 1 stage for edge detection)
  reg [2:0] csn_sync;
  reg [2:0] sck_sync;
  reg [1:0] mosi_sync;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      csn_sync  <= 3'b111;
      sck_sync  <= 3'b000;
      mosi_sync <= 2'b00;
    end else begin
      csn_sync  <= {csn_sync[1:0], spi_cs_n};
      sck_sync  <= {sck_sync[1:0], spi_sck};
      mosi_sync <= {mosi_sync[0], spi_mosi};
    end
  end

  wire csn_val  = csn_sync[1];
  wire csn_fall = (csn_sync[2:1] == 2'b10);
  wire csn_rise = (csn_sync[2:1] == 2'b01);
  wire sck_rise = (sck_sync[2:1] == 2'b01) && (!csn_val);
  wire sck_fall = (sck_sync[2:1] == 2'b10) && (!csn_val);
  wire mosi_val = mosi_sync[1];

  assign frame_start = csn_fall;
  assign frame_end   = csn_rise;

  reg [2:0] bit_cnt;
  reg [7:0] rx_shift;
  reg [7:0] tx_shift;
  reg       byte_done_pending;

  assign spi_miso = csn_val ? 1'b0 : tx_shift[7];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_cnt           <= 3'd0;
      rx_shift          <= 8'h00;
      tx_shift          <= 8'h00;
      rx_valid          <= 1'b0;
      rx_byte           <= 8'h00;
      frame_err         <= 1'b0;
      byte_done_pending <= 1'b0;
    end else begin
      rx_valid  <= 1'b0;
      frame_err <= 1'b0;

      if (csn_fall) begin
        bit_cnt           <= 3'd0;
        tx_shift          <= tx_byte;
        byte_done_pending <= 1'b0;
      end else if (csn_rise) begin
        if (bit_cnt != 3'd0) begin
          frame_err <= 1'b1;
        end
        bit_cnt           <= 3'd0;
        byte_done_pending <= 1'b0;
      end else if (!csn_val) begin
        if (sck_rise) begin
          rx_shift <= {rx_shift[6:0], mosi_val};
          if (bit_cnt == 3'd7) begin
            bit_cnt           <= 3'd0;
            rx_byte           <= {rx_shift[6:0], mosi_val};
            rx_valid          <= 1'b1;
            byte_done_pending <= 1'b1;
          end else begin
            bit_cnt <= bit_cnt + 3'd1;
          end
        end else if (sck_fall) begin
          if (byte_done_pending) begin
            tx_shift          <= tx_byte;
            byte_done_pending <= 1'b0;
          end else begin
            tx_shift <= {tx_shift[6:0], 1'b0};
          end
        end
      end
    end
  end

endmodule
