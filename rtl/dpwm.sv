// -----------------------------------------------------------------------------
// dpwm.sv : digital PWM for a synchronous buck stage (synthesizable)
//   * trailing-edge modulation, free-running period counter (N clk cycles)
//   * programmable dead time between HS and LS (guaranteed by construction)
//   * cycle-by-cycle peak-current limit: ocp_trip truncates the HS on-time
//   * duty dithering: duty_q has DUTY_FRAC fractional bits; the fraction is
//     accumulated period-to-period (1st-order dither) for finer effective
//     resolution than the clk/fsw ratio allows
//   * sample_tick / period_start strobes for the ADC and control loop
// -----------------------------------------------------------------------------
module dpwm #(
  parameter int N          = 800,   // clk cycles per switching period
  parameter int DT         = 8,     // dead time in clk cycles
  parameter int DUTY_FRAC  = 3,     // fractional duty bits (dither)
  parameter int DUTY_W     = 13,    // duty word width: Q(DUTY_W-DUTY_FRAC).DUTY_FRAC
  parameter int SAMPLE_CNT = 752    // counter value at which the ADC is triggered
) (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              enable,
  input  logic [DUTY_W-1:0] duty_q,        // duty command, counts * 2^DUTY_FRAC
  input  logic              ocp_trip,      // peak-current comparator (synchronized)
  output logic              hs,
  output logic              ls,
  output logic              sample_tick,   // 1-clk pulse when cnt == SAMPLE_CNT
  output logic              period_start   // 1-clk pulse when cnt == 0
);
  localparam int CW   = $clog2(N);
  localparam int INTW = DUTY_W - DUTY_FRAC;
  localparam int TW   = $clog2(DT + 1) + 1;

  logic [CW-1:0]        cnt;
  logic [DUTY_FRAC-1:0] dacc;
  logic [INTW-1:0]      duty_int;
  logic [TW-1:0]        off_tmr;      // clk cycles since HS last on (saturates at DT)
  logic                 trunc;        // HS already cut this period by current limit
  logic                 hs_next, ls_next;
  logic [DUTY_FRAC:0]   dsum;

  wire wrap = (cnt == CW'(N - 1));

  always_comb begin
    dsum    = {1'b0, dacc} + {1'b0, duty_q[DUTY_FRAC-1:0]};
    hs_next = enable && !trunc && !ocp_trip && (cnt < duty_int);
    // LS only after HS has been off for DT cycles, and off again DT cycles
    // before the next period starts (so HS turn-on never overlaps LS)
    ls_next = enable && !hs_next && !hs && (off_tmr >= TW'(DT)) && (cnt < CW'(N - DT));
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cnt <= '0; dacc <= '0; duty_int <= '0; off_tmr <= '0; trunc <= 1'b0;
      hs <= 1'b0; ls <= 1'b0; sample_tick <= 1'b0; period_start <= 1'b0;
    end else begin
      cnt          <= wrap ? '0 : cnt + 1'b1;
      sample_tick  <= (cnt == CW'(SAMPLE_CNT - 1));
      period_start <= wrap;

      if (wrap) begin                         // latch next period's duty (+ dither)
        trunc <= 1'b0;
        if (enable) begin
          dacc     <= dsum[DUTY_FRAC-1:0];
          duty_int <= duty_q[DUTY_W-1:DUTY_FRAC] + INTW'(dsum[DUTY_FRAC]);
        end else begin
          dacc <= '0; duty_int <= '0;
        end
      end else if (ocp_trip && hs) begin
        trunc <= 1'b1;
      end

      hs <= hs_next;
      ls <= ls_next;
      if (hs)                          off_tmr <= '0;
      else if (off_tmr < TW'(DT))      off_tmr <= off_tmr + 1'b1;
    end
  end
endmodule
