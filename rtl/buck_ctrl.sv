// -----------------------------------------------------------------------------
// buck_ctrl.sv : digital voltage-mode controller for a synchronous buck
//
//   IDLE -> CAL -> SS -> REG            (any run state -> IDLE on !en / !vin_ok)
//                  \-> FAULT (latched, leave with fault_clear)
//
//   CAL   : two measurements, deliberately kept separate
//           1) ADC auto-zero: the ADC input is muxed to ground (adc_zero), so the
//              reading is purely the ADC offset -> cal_offset (> CAL_MAX: CAL fault)
//           2) pre-bias / discharge check: with the offset known, the corrected
//              output must be <= DISCH_CODE before the converter may start (e.g.
//              restart right after UVLO). Re-checked up to CAL_RETRIES times while
//              the output decays, then PREBIAS fault. Starting into a charged
//              output would otherwise make the loop sink current from the output.
//           (cal_en = 0 skips step 1 and leaves the offset uncorrected.)
//   SS    : reference ramps 0 -> vref over SS_PERIODS switching periods,
//           PID closes the loop during the ramp.
//   REG   : regulating; PGOOD with debounce + hysteresis.
//   Protection (SS and REG):
//           OCP  - peak-current comparator; cycle-by-cycle limit inside dpwm,
//                  OCP_LIMIT consecutive limited periods -> latched fault
//           OVP  - corrected ADC > OVP_CODE for OVP_DEB samples
//           UVP  - corrected ADC < UVP_CODE for UVP_DEB samples (REG only)
//
//   One ADC sample per switching period (trigger from dpwm.sample_tick, late in
//   the period) so the loop's sample->duty latency is a fraction of a period.
// -----------------------------------------------------------------------------
module buck_ctrl #(
  parameter int N          = 800,
  parameter int DT         = 8,
  parameter int SAMPLE_CNT = 752,
  parameter int DUTY_MAX   = 720,     // counts (90 %)
  parameter int SS_PERIODS = 100,
  parameter int CAL_SKIP   = 4,       // periods ignored before averaging
  parameter int CAL_LOG2   = 4,       // average 2^CAL_LOG2 samples
  parameter int CAL_MAX    = 64,      // max tolerated ADC offset [codes]
  parameter int CAL_RETRIES = 15,     // discharge re-checks before a PREBIAS fault
  parameter int DISCH_CODE  = 24,     // output must read <= this (~47 mV) to start
  parameter int OVP_CODE   = 675,     // ~1.32 V  (+10 %)
  parameter int UVP_CODE   = 430,     // ~0.84 V  (70 %)
  parameter int OVP_DEB    = 2,
  parameter int UVP_DEB    = 4,
  parameter int OCP_LIMIT  = 8,
  parameter int PG_WIN     = 15,      // PGOOD window [codes]
  parameter int PG_HYST    = 30,      // PGOOD drop threshold [codes]
  parameter int PG_DEB     = 32
) (
  input  logic               clk,
  input  logic               rst_n,
  // control / status
  input  logic               en,
  input  logic               vin_ok,
  input  logic               cal_en,
  input  logic               fault_clear,
  input  logic [9:0]         vref_code,
  input  logic signed [19:0] kp,
  input  logic signed [19:0] ki,
  input  logic signed [19:0] kd,
  output logic               pgood,
  output logic [2:0]         state,
  output logic [2:0]         fault_code,
  output logic signed [10:0] cal_offset,
  // analog front-end interface
  output logic               adc_start,
  output logic               adc_zero,       // 1: ADC input muxed to ground (auto-zero)
  input  logic [9:0]         adc_data,
  input  logic               adc_valid,
  input  logic               ocp_comp,
  // gate drive
  output logic               hs,
  output logic               ls,
  // debug
  output logic [12:0]        duty_dbg,
  output logic [9:0]         ref_dbg
);
  localparam int DUTY_FRAC  = 3;
  localparam int DUTY_W     = 13;
  localparam int DUTY_MAX_Q = DUTY_MAX * (1 << DUTY_FRAC);

  localparam logic [2:0] S_IDLE = 3'd0, S_CAL = 3'd1, S_SS = 3'd2, S_REG = 3'd3, S_FAULT = 3'd4;
  localparam logic [2:0] F_NONE = 3'd0, F_OCP = 3'd1, F_OVP = 3'd2, F_UVP = 3'd3, F_CAL = 3'd4, F_PREBIAS = 3'd5;

  wire run = (state == S_SS) || (state == S_REG);

  // ---- comparator synchronizer ---------------------------------------------
  logic ocp_s1, ocp_sync;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin ocp_s1 <= 1'b0; ocp_sync <= 1'b0; end
    else        begin ocp_s1 <= ocp_comp; ocp_sync <= ocp_s1; end

  // ---- datapath blocks ------------------------------------------------------
  logic        sample_tick, period_start;
  logic [12:0] duty_q;
  logic        sat_hi, sat_lo, pid_upd;
  logic signed [11:0] err_r;

  dpwm #(.N(N), .DT(DT), .DUTY_FRAC(DUTY_FRAC), .DUTY_W(DUTY_W), .SAMPLE_CNT(SAMPLE_CNT)) u_dpwm (
    .clk(clk), .rst_n(rst_n), .enable(run), .duty_q(duty_q), .ocp_trip(ocp_sync),
    .hs(hs), .ls(ls), .sample_tick(sample_tick), .period_start(period_start));
  assign adc_start = sample_tick;

  pid_ctrl #(.EW(12), .KW(20), .FRAC(12), .DUTY_W(DUTY_W), .DUTY_FRAC(DUTY_FRAC), .DUTY_MAX_Q(DUTY_MAX_Q)) u_pid (
    .clk(clk), .rst_n(rst_n), .clear(!run), .update(pid_upd), .err(err_r),
    .kp(kp), .ki(ki), .kd(kd), .duty_q(duty_q), .sat_hi(sat_hi), .sat_lo(sat_lo));

  // ---- calibration helpers --------------------------------------------------
  logic [4:0]  cal_cnt;
  logic [3:0]  cal_try;
  logic [15:0] cal_sum;
  wire  [15:0] cal_sum_f = cal_sum + {6'b0, adc_data};
  wire  [15:0] cal_avg   = cal_sum_f >> CAL_LOG2;
  logic        cal_ph;                                 // 0: auto-zero, 1: discharge check
  assign adc_zero = (state == S_CAL) && !cal_ph;
  wire signed [11:0] vo_res = $signed({2'b00, cal_avg[9:0]}) - cal_offset;   // residual output (phase 1)

  // ---- reference ramp / error pipeline -------------------------------------
  logic [15:0] ref_q;                                  // Q10.6 codes
  wire  [15:0] ref_target = {vref_code, 6'b0};
  wire  [15:0] ss_step    = 16'(ref_target / SS_PERIODS);
  logic [9:0]  adc_r;
  logic        vB;
  wire signed [11:0] corr_w = $signed({2'b00, adc_r}) - cal_offset;       // corrected feedback
  wire signed [11:0] err_w  = $signed({2'b00, ref_q[15:6]}) - corr_w;     // ref - feedback
  wire        [11:0] abs_err = err_w[11] ? (~err_w + 12'd1) : err_w;

  assign ref_dbg  = ref_q[15:6];
  assign duty_dbg = duty_q;

  // ---- protection counters --------------------------------------------------
  logic       ocp_seen;
  logic [3:0] ocp_cnt, ovp_cnt, uvp_cnt;
  logic [5:0] pg_cnt;

  wire ovp_hit = (corr_w > OVP_CODE);
  wire uvp_hit = (state == S_REG) && (corr_w < UVP_CODE);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= S_IDLE; fault_code <= F_NONE; cal_offset <= '0; pgood <= 1'b0;
      cal_cnt <= '0; cal_try <= '0; cal_sum <= '0; cal_ph <= 1'b0; ref_q <= '0; adc_r <= '0; vB <= 1'b0; pid_upd <= 1'b0; err_r <= '0;
      ocp_seen <= 1'b0; ocp_cnt <= '0; ovp_cnt <= '0; uvp_cnt <= '0; pg_cnt <= '0;
    end else begin
      vB <= 1'b0; pid_upd <= 1'b0;
      if (ocp_sync && run) ocp_seen <= 1'b1;

      case (state)
        // --------------------------------------------------------------------
        S_IDLE: begin
          ref_q <= '0; cal_cnt <= '0; cal_try <= '0; cal_sum <= '0; pgood <= 1'b0; ocp_seen <= 1'b0;
          cal_ph <= !cal_en; cal_offset <= '0;
          ocp_cnt <= '0; ovp_cnt <= '0; uvp_cnt <= '0; pg_cnt <= '0;
          if (en && vin_ok) state <= S_CAL;
        end
        // --------------------------------------------------------------------
        S_CAL: begin
          if (!en || !vin_ok) state <= S_IDLE;
          else if (adc_valid) begin
            if (cal_cnt < CAL_SKIP) cal_cnt <= cal_cnt + 1'b1;
            else if (cal_cnt < CAL_SKIP + (1 << CAL_LOG2) - 1) begin
              cal_sum <= cal_sum + {6'b0, adc_data};
              cal_cnt <= cal_cnt + 1'b1;
            end else begin                                  // last averaging sample
              if (!cal_ph) begin                            // ---- ADC auto-zero result
                if (cal_avg > CAL_MAX) begin
                  state <= S_FAULT; fault_code <= F_CAL;
                end else begin
                  cal_offset <= cal_avg[10:0]; cal_ph <= 1'b1; cal_cnt <= '0; cal_sum <= '0;
                end
              end else begin                                // ---- discharge / pre-bias check
                if (vo_res <= DISCH_CODE) state <= S_SS;
                else if (cal_try < CAL_RETRIES) begin
                  cal_try <= cal_try + 1'b1; cal_cnt <= '0; cal_sum <= '0;
                end else begin
                  state <= S_FAULT; fault_code <= F_PREBIAS;
                end
              end
            end
          end
        end
        // --------------------------------------------------------------------
        S_SS, S_REG: begin
          if (!en || !vin_ok) begin
            state <= S_IDLE;                                // UVLO / disable: clean restart later
          end else begin
            if (adc_valid) begin
              adc_r <= adc_data; vB <= 1'b1;
              if (state == S_SS) begin
                if ((ref_q + ss_step) >= ref_target) begin ref_q <= ref_target; state <= S_REG; end
                else                                  ref_q <= ref_q + ss_step;
              end else ref_q <= ref_target;
            end
            if (vB) begin                                   // 1 clk after adc_valid
              err_r   <= err_w;
              pid_upd <= 1'b1;
              ocp_seen <= ocp_sync;
              ovp_cnt <= ovp_hit  ? ovp_cnt + 1'b1 : 4'd0;
              ocp_cnt <= ocp_seen ? ocp_cnt + 1'b1 : 4'd0;
              uvp_cnt <= uvp_hit  ? uvp_cnt + 1'b1 : 4'd0;
              if      (ovp_hit  && (ovp_cnt + 1 >= OVP_DEB))   begin state <= S_FAULT; fault_code <= F_OVP; pgood <= 1'b0; end
              else if (ocp_seen && (ocp_cnt + 1 >= OCP_LIMIT)) begin state <= S_FAULT; fault_code <= F_OCP; pgood <= 1'b0; end
              else if (uvp_hit  && (uvp_cnt + 1 >= UVP_DEB))   begin state <= S_FAULT; fault_code <= F_UVP; pgood <= 1'b0; end
              // PGOOD (only meaningful in REG)
              if (state == S_REG) begin
                if (abs_err <= PG_WIN) begin
                  if (pg_cnt + 1 >= PG_DEB) pgood <= 1'b1;
                  if (pg_cnt < PG_DEB) pg_cnt <= pg_cnt + 1'b1;
                end else begin
                  pg_cnt <= '0;
                  if (abs_err > PG_HYST) pgood <= 1'b0;
                end
              end
            end
          end
        end
        // --------------------------------------------------------------------
        S_FAULT: begin
          pgood <= 1'b0;
          if (fault_clear) begin state <= S_IDLE; fault_code <= F_NONE; end
        end
        default: state <= S_IDLE;
      endcase
    end
  end
endmodule
