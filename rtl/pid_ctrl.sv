// -----------------------------------------------------------------------------
// pid_ctrl.sv : fixed-point digital PID compensator (synthesizable)
//   Gc(z) = Kp + Ki/(1 - z^-1) + Kd*(1 - z^-1)         (gains in Q.FRAC)
//   * input  : err  = ref - feedback, in ADC LSBs (signed)
//   * output : duty_q in DPWM counts * 2^DUTY_FRAC, clamped to [0, DUTY_MAX_Q]
//   * anti-windup: integrator is clamped to the duty range and frozen while the
//     output is saturated and the error would push it further into saturation
// -----------------------------------------------------------------------------
module pid_ctrl #(
  parameter int EW         = 12,
  parameter int KW         = 20,
  parameter int FRAC       = 12,
  parameter int DUTY_W     = 13,
  parameter int DUTY_FRAC  = 3,
  parameter int DUTY_MAX_Q = 5760          // 720 counts * 2^DUTY_FRAC
) (
  input  logic                 clk,
  input  logic                 rst_n,
  input  logic                 clear,       // synchronous reset of the dynamic state
  input  logic                 update,      // 1-clk pulse: err is valid, compute new duty
  input  logic signed [EW-1:0] err,
  input  logic signed [KW-1:0] kp,
  input  logic signed [KW-1:0] ki,
  input  logic signed [KW-1:0] kd,
  output logic [DUTY_W-1:0]    duty_q,
  output logic                 sat_hi,
  output logic                 sat_lo
);
  localparam int AW = 40;
  localparam logic signed [AW-1:0] I_MAX = DUTY_MAX_Q * (1 << (FRAC - DUTY_FRAC));

  logic signed [EW-1:0] e_prev;
  logic signed [AW-1:0] acc;
  logic signed [AW-1:0] p_term, i_inc, d_term, acc_try, acc_next, u_sum, u_q;
  logic                 hold;

  always_comb begin
    p_term   = kp * err;
    i_inc    = ki * err;
    d_term   = kd * (err - e_prev);
    hold     = (sat_hi && (err > 0)) || (sat_lo && (err < 0));
    acc_try  = hold ? acc : (acc + i_inc);
    acc_next = (acc_try < 0) ? '0 : (acc_try > I_MAX) ? I_MAX : acc_try;
    u_sum    = p_term + acc_next + d_term;
    u_q      = u_sum >>> (FRAC - DUTY_FRAC);           // -> duty counts * 2^DUTY_FRAC
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      acc <= '0; e_prev <= '0; duty_q <= '0; sat_hi <= 1'b0; sat_lo <= 1'b0;
    end else if (clear) begin                       // synchronous clear
      acc <= '0; e_prev <= '0; duty_q <= '0; sat_hi <= 1'b0; sat_lo <= 1'b0;
    end else if (update) begin
      acc    <= acc_next;
      e_prev <= err;
      sat_hi <= (u_q > DUTY_MAX_Q);
      sat_lo <= (u_q < 0);
      duty_q <= (u_q < 0) ? '0 : (u_q > DUTY_MAX_Q) ? DUTY_W'(DUTY_MAX_Q) : u_q[DUTY_W-1:0];
    end
  end
endmodule
