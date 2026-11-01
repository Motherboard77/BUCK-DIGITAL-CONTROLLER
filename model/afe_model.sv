// -----------------------------------------------------------------------------
// afe_model.sv : analog front-end behavioral model (simulation only)
//   * 10-bit ADC, full scale ADC_FS, sample-and-hold at `start`, conversion
//     latency LAT clk cycles, optional uniform noise, injectable offset error
//     (offset_codes, in LSBs) used to exercise calibration and fault paths
//   * zero_sel muxes the ADC input to ground so the DUT can measure the offset alone
//   * peak-current comparator: ocp_comp = (il > ILIM), 1 clk delay
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module afe_model #(
  parameter real ADC_FS    = 2.0,
  parameter int  LAT       = 16,
  parameter real NOISE_LSB = 0.3,      // peak of uniform noise, in LSBs
  parameter real ILIM      = 9.0
) (
  input  logic        clk,
  input  logic        start,
  input  logic        zero_sel,       // 1: input shorted to ground (auto-zero)
  input  real         vo,
  input  real         il,
  input  real         offset_codes,
  output logic [9:0]  data,
  output logic        valid,
  output logic        ocp_comp
);
  integer seed;
  integer lat_cnt;
  real    s, rnd, code_r;
  integer code_i;

  initial begin data = 10'd0; valid = 1'b0; ocp_comp = 1'b0; lat_cnt = 0; seed = 32'h1234_5678; end

  always @(posedge clk) begin
    ocp_comp <= (il > ILIM);
    valid    <= 1'b0;

    if (start) begin
      rnd    = (($random(seed) & 32'hFFFF) / 65535.0 - 0.5) * 2.0 * NOISE_LSB;
      code_r = ((zero_sel ? 0.0 : vo) / ADC_FS) * 1024.0 + offset_codes + rnd;
      code_i = $rtoi(code_r + 0.5);
      if (code_r + 0.5 < 0.0) code_i = 0;
      if (code_i > 1023) code_i = 1023;
      if (code_i < 0)    code_i = 0;
      s       = code_i;
      lat_cnt = LAT;
    end else if (lat_cnt > 0) begin
      lat_cnt = lat_cnt - 1;
      if (lat_cnt == 0) begin
        data  <= $rtoi(s);
        valid <= 1'b1;
      end
    end
  end
endmodule
