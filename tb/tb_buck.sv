// -----------------------------------------------------------------------------
// tb_buck.sv : closed-loop mixed-signal testbench (RTL controller + behavioral
//              analog plant, one simulator, fixed 2.5 ns step = clk)
//
//   +SCEN=main     startup, steady state, load step 2 A -> 3 A, load release
//   +SCEN=nocal    same, calibration bypassed (negative control: offset shows up)
//   +SCEN=calfail  ADC offset beyond CAL_MAX -> CAL fault, gates never switch
//   +SCEN=ocp      overload (10 A demand > 9 A limit, output stays > UVP) -> OCP latch, then restart
//   +SCEN=short    hard output short -> cycle-by-cycle limit, fast latched fault, then restart
//   +SCEN=ovp      sense-path fault injection (reads high) -> OVP, then restart
//   +SCEN=uvp      sense-path fault injection (reads low)  -> UVP, then restart
//   +SCEN=uvlo     input UVLO drop -> clean shutdown, automatic restart
//   +ADC_OFF=<n>   injected ADC offset in LSBs (default 10 = 19.5 mV)
// Prints CHECK lines + a final RESULT line; writes results/<scen>*.csv
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_buck;
  // ------------------------------ configuration ------------------------------
  localparam real VIN_NOM  = 5.0;
  localparam real VREF_V   = 1.2;
  localparam real LSB_V    = 2.0/1024.0;
  localparam real R_LIGHT  = 0.6;                              // 2 A
  localparam real R_PAR    = 1.2;                              // switched in parallel for the step
  localparam real R_HEAVY  = (R_LIGHT*R_PAR)/(R_LIGHT+R_PAR);  // 3 A
  localparam real ILIM     = 9.0;
  localparam signed [19:0] KP = 20'sd4096;    // 1.0   (Q.12)  -- see scripts/design_compensator.py
  localparam signed [19:0] KI = 20'sd410;     // 0.1
  localparam signed [19:0] KD = 20'sd32768;   // 8.0

  localparam real T_EN = 20.0, T_STEP = 600.0, T_REL = 900.0, T_END_MAIN = 1200.0;

  // --------------------------------- signals ---------------------------------
  logic clk = 1'b0;
  always #1.25 clk = ~clk;                                     // 400 MHz

  logic rst_n, en, vin_ok, cal_en, fault_clear;
  logic [9:0] vref_code;
  logic pgood; logic [2:0] state, fault_code; logic signed [10:0] cal_offset;
  logic adc_start, adc_zero, adc_valid, ocp_comp, hs, ls;
  logic [9:0] adc_data; logic [12:0] duty_dbg; logic [9:0] ref_dbg;
  real vin, rload, il, vo, vsw, adc_offset;

  buck_ctrl dut (
    .clk, .rst_n, .en, .vin_ok, .cal_en, .fault_clear, .vref_code,
    .kp(KP), .ki(KI), .kd(KD),
    .pgood, .state, .fault_code, .cal_offset,
    .adc_start, .adc_zero, .adc_data, .adc_valid, .ocp_comp, .hs, .ls, .duty_dbg, .ref_dbg);

  buck_plant plant (.clk, .hs, .ls, .vin, .rload, .il, .vo, .vsw);

  afe_model #(.ILIM(ILIM)) afe (
    .clk, .start(adc_start), .zero_sel(adc_zero), .vo, .il, .offset_codes(adc_offset),
    .data(adc_data), .valid(adc_valid), .ocp_comp);

  // --------------------------------- helpers ---------------------------------
  string scen;
  integer adc_off_cfg;
  integer n_fail = 0, n_pass = 0;
  real t_us;
  always @(posedge clk) t_us = $realtime / 1000.0;

  task automatic wait_us(input real t_target);
    real d;
    begin
      d = t_target*1000.0 - $realtime;
      if (d > 0.0) #(d);
    end
  endtask

  task automatic check(input string name, input bit ok, input real val);
    begin
      if (ok) begin n_pass = n_pass + 1; $display("CHECK  PASS  %-42s (%0.4f)", name, val); end
      else    begin n_fail = n_fail + 1; $display("CHECK  FAIL  %-42s (%0.4f)", name, val); end
    end
  endtask

  // -------------------------------- stimulus ---------------------------------
  initial begin
    if (!$value$plusargs("SCEN=%s", scen)) scen = "main";
    if (!$value$plusargs("ADC_OFF=%d", adc_off_cfg)) adc_off_cfg = 10;
    if (scen == "calfail") adc_off_cfg = 100;
    rst_n = 1'b0; en = 1'b0; vin_ok = 1'b1; fault_clear = 1'b0; vref_code = 10'd614;
    cal_en = (scen != "nocal");
    vin = VIN_NOM; rload = R_LIGHT; adc_offset = adc_off_cfg;
    repeat (20) @(posedge clk);
    rst_n = 1'b1;
    wait_us(T_EN); en = 1'b1;

    if (scen == "main" || scen == "nocal") begin
      wait_us(T_STEP); rload = R_HEAVY;
      wait_us(T_REL);  rload = R_LIGHT;
      wait_us(T_END_MAIN);
    end else if (scen == "ocp" || scen == "short") begin
      wait_us(700);  rload = (scen == "ocp") ? 0.12 : 0.01;   // overload (10 A @ 1.2 V) / hard short
      wait_us(900);  rload = R_LIGHT;               // fault removed
      wait_us(950);  fault_clear = 1'b1; @(posedge clk); fault_clear = 1'b0;
      wait_us(1450);
    end else if (scen == "ovp" || scen == "uvp") begin
      wait_us(700);  adc_offset = adc_off_cfg + ((scen == "ovp") ? 400.0 : -400.0);
      wait_us(800);  adc_offset = adc_off_cfg;
      wait_us(900);  fault_clear = 1'b1; @(posedge clk); fault_clear = 1'b0;
      wait_us(1450);
    end else if (scen == "uvlo") begin
      wait_us(700);  vin_ok = 1'b0;
      wait_us(800);  vin_ok = 1'b1;     // output still discharging -> controller must wait, then soft-start
      wait_us(1700);
    end else if (scen == "calfail") begin
      wait_us(600);
    end
    evaluate();
    $finish;
  end

  // --------------------------------- monitors ---------------------------------
  // always-on
  real il_max = 0.0, il_min = 0.0;
  integer shoot_cnt = 0, ocp_events = 0, gate_act_after_fault = 0, gate_act_total = 0;
  real t_fault = -1.0, t_pgood = -1.0, t_idle_dn = -1.0;
  logic ocp_q = 1'b0; logic [2:0] state_q = 3'd0;
  // windows
  real su_vmax = 0.0, su_imax = 0.0;                         // startup
  real w1_sum = 0.0; integer w1_n = 0; real w1_vmin = 9.0, w1_vmax = 0.0, w1_imin = 99.0, w1_imax = -99.0;
  real st_vmin = 9.0, st_last_out = 0.0;                     // load step
  real rl_vmax = 0.0, rl_last_out = 0.0;                     // load release
  real wf_sum = 0.0; integer wf_n = 0; logic pg_at_end = 1'b0;   // final window
  real T_W1_A, T_W1_B, T_WF_A, T_WF_B;
  integer cal_offset_seen = -1;
  // first fault code seen (the code is cleared again by the restart after fault_clear)
  logic [2:0] first_fault_code = 3'd0;
  always @(posedge clk) if (state == 3'd4 && first_fault_code == 3'd0) first_fault_code <= fault_code;
  real hold_vo_min = 9.0;

  initial begin
    if (scen == "main" || scen == "nocal") begin T_W1_A = 550.0; T_W1_B = 600.0; T_WF_A = 1150.0; T_WF_B = 1200.0; end
    else                                   begin T_W1_A = 650.0; T_W1_B = 700.0; T_WF_A = 1400.0; T_WF_B = 1450.0; end
    if (scen == "uvlo")                    begin T_WF_A = 1650.0; T_WF_B = 1700.0; end
  end

  always @(posedge clk) if (rst_n) begin
    if (il > il_max) il_max = il;
    if (il < il_min) il_min = il;
    if (hs && ls) shoot_cnt = shoot_cnt + 1;
    if (ocp_comp && !ocp_q) ocp_events = ocp_events + 1;
    ocp_q <= ocp_comp;
    if (hs || ls) gate_act_total = gate_act_total + 1;
    if (state == 3'd4 && state_q != 3'd4 && t_fault < 0.0) t_fault = t_us;
    if (state == 3'd0 && state_q != 3'd0 && t_idle_dn < 0.0 && t_us > 100.0) t_idle_dn = t_us;
    if (pgood && t_pgood < 0.0) t_pgood = t_us;
    state_q <= state;
    if (state == 3'd2 && cal_offset_seen < 0) cal_offset_seen = cal_offset;
    // gate activity once the fault has been latched (allow 1 us for the shutdown)
    if (state == 3'd4 && t_fault > 0.0 && t_us > t_fault + 1.0 && (hs || ls)) gate_act_after_fault = gate_act_after_fault + 1;

    if (t_us >= T_EN && t_us < T_STEP) begin
      if (vo > su_vmax) su_vmax = vo;
      if (il > su_imax) su_imax = il;
    end
    if (t_us >= T_W1_A && t_us < T_W1_B) begin
      w1_sum = w1_sum + vo; w1_n = w1_n + 1;
      if (vo < w1_vmin) w1_vmin = vo;  if (vo > w1_vmax) w1_vmax = vo;
      if (il < w1_imin) w1_imin = il;  if (il > w1_imax) w1_imax = il;
    end
    if (t_us >= T_STEP && t_us < T_REL) begin
      if (vo < st_vmin) st_vmin = vo;
      if (vo > VREF_V*1.015 || vo < VREF_V*0.985) st_last_out = t_us;
    end
    if (t_us >= T_REL && t_us < T_END_MAIN) begin
      if (vo > rl_vmax) rl_vmax = vo;
      if (vo > VREF_V*1.015 || vo < VREF_V*0.985) rl_last_out = t_us;
    end
    if (t_us >= T_WF_A && t_us < T_WF_B) begin
      wf_sum = wf_sum + vo; wf_n = wf_n + 1; pg_at_end = pgood;
    end
  end

  // ---------------------------------- logging ---------------------------------
  integer f_wave, f_edge, f_zoom, lg_cnt = 0;
  initial begin
    #1;
    f_wave = $fopen($sformatf("results/%0s.csv", scen), "w");
    f_edge = $fopen($sformatf("results/%0s_edges.csv", scen), "w");
    f_zoom = $fopen($sformatf("results/%0s_zoom.csv", scen), "w");
    $fdisplay(f_wave, "t_us,vo,il,duty_counts,ref_v,state,fault,pgood,rload,hs,ls");
    $fdisplay(f_zoom, "t_us,vo,il,hs,ls");
    $fdisplay(f_edge, "t_ns,hs,ls,rload,vin");
  end
  always @(posedge clk) if (rst_n) begin
    lg_cnt = lg_cnt + 1;
    if (lg_cnt % 20 == 0)
      $fdisplay(f_wave, "%0.3f,%0.5f,%0.4f,%0.3f,%0.5f,%0d,%0d,%0d,%0.4f,%0d,%0d",
                t_us, vo, il, duty_dbg/8.0, ref_dbg*LSB_V, state, fault_code, pgood, rload, hs, ls);
    if (t_us >= 580.0 && t_us < 584.0)
      $fdisplay(f_zoom, "%0.4f,%0.6f,%0.5f,%0d,%0d", t_us, vo, il, hs, ls);
  end
  logic hs_q = 1'b0, ls_q = 1'b0; real rload_q = 0.0, vin_q = 0.0;
  always @(posedge clk) if (rst_n) begin
    if (hs !== hs_q || ls !== ls_q || rload != rload_q || vin != vin_q)
      $fdisplay(f_edge, "%0.2f,%0d,%0d,%0.4f,%0.3f", $realtime, hs, ls, rload, vin);
    hs_q <= hs; ls_q <= ls; rload_q <= rload; vin_q <= vin;
  end

  // --------------------------------- evaluation -------------------------------
  real w1_avg, wf_avg, ripple_v, ripple_i;
  task automatic evaluate;
    begin
      w1_avg   = (w1_n > 0) ? w1_sum / w1_n : 0.0;
      wf_avg   = (wf_n > 0) ? wf_sum / wf_n : 0.0;
      ripple_v = w1_vmax - w1_vmin;
      ripple_i = w1_imax - w1_imin;
      $display("---------------- scenario: %0s  (ADC offset %0d LSB) ----------------", scen, adc_off_cfg);
      $display("METRIC il_max_A %0.3f",              il_max);
      $display("METRIC shoot_through_cycles %0d",    shoot_cnt);
      check("no shoot-through (HS & LS never both on)", shoot_cnt == 0, shoot_cnt);

      if (scen == "main") begin
        $display("METRIC cal_offset_codes %0d",        cal_offset_seen);
        $display("METRIC startup_overshoot_pct %0.3f", (su_vmax - VREF_V)/VREF_V*100.0);
        $display("METRIC startup_peak_il_A %0.3f",     su_imax);
        $display("METRIC t_pgood_us %0.2f",            t_pgood);
        $display("METRIC vo_ss_avg_V %0.5f",           w1_avg);
        $display("METRIC vo_ss_err_pct %0.3f",         (w1_avg - VREF_V)/VREF_V*100.0);
        $display("METRIC vo_ripple_pp_mV %0.3f",       ripple_v*1000.0);
        $display("METRIC il_ripple_pp_A %0.3f",        ripple_i);
        $display("METRIC step_droop_mV %0.3f",         (VREF_V - st_vmin)*1000.0);
        $display("METRIC step_recovery_us %0.2f",      st_last_out - T_STEP);
        $display("METRIC release_overshoot_mV %0.3f",  (rl_vmax - VREF_V)*1000.0);
        $display("METRIC release_recovery_us %0.2f",   rl_last_out - T_REL);
        $display("METRIC vo_final_avg_V %0.5f",        wf_avg);
        check("calibration measured injected offset (+/-1)", (cal_offset_seen >= adc_off_cfg-1) && (cal_offset_seen <= adc_off_cfg+1), cal_offset_seen);
        check("startup overshoot < 3 %",                  su_vmax < VREF_V*1.03, (su_vmax-VREF_V)/VREF_V*100.0);
        check("startup peak inductor current < ILIM",     su_imax < ILIM, su_imax);
        check("no current-limit events during normal run",ocp_events == 0, ocp_events);
        check("no fault latched during normal run",       t_fault < 0.0, t_fault);
        check("PGOOD asserted before load step",          t_pgood > 0.0 && t_pgood < T_STEP, t_pgood);
        check("DC accuracy within +/-1 % (before step)",  (w1_avg > VREF_V*0.99) && (w1_avg < VREF_V*1.01), (w1_avg-VREF_V)/VREF_V*100.0);
        check("load-step droop < 6 %",                    st_vmin > VREF_V*0.94, (VREF_V-st_vmin)/VREF_V*100.0);
        check("load-step recovery to +/-1.5 % < 150 us",  (st_last_out - T_STEP) < 150.0, st_last_out - T_STEP);
        check("load-release overshoot < 6 %",             rl_vmax < VREF_V*1.06, (rl_vmax-VREF_V)/VREF_V*100.0);
        check("load-release recovery to +/-1.5 % < 150 us",(rl_last_out - T_REL) < 150.0, rl_last_out - T_REL);
        check("final DC accuracy within +/-1 %",          (wf_avg > VREF_V*0.99) && (wf_avg < VREF_V*1.01), (wf_avg-VREF_V)/VREF_V*100.0);
      end

      else if (scen == "nocal") begin
        $display("METRIC vo_ss_avg_V %0.5f",   w1_avg);
        $display("METRIC vo_ss_err_pct %0.3f", (w1_avg - VREF_V)/VREF_V*100.0);
        check("negative control: uncalibrated error exceeds 1 %", (w1_avg < VREF_V*0.99) || (w1_avg > VREF_V*1.01), (w1_avg-VREF_V)/VREF_V*100.0);
        check("negative control: still regulates (no fault)",     t_fault < 0.0, t_fault);
      end

      else if (scen == "calfail") begin
        check("CAL fault latched",                  fault_code == 3'd4 && state == 3'd4, fault_code);
        check("gates never switched",               gate_act_total == 0, gate_act_total);
        check("output stays at 0 V",                il_max < 0.01, il_max);
      end

      else if (scen == "ocp" || scen == "short") begin
        $display("METRIC t_fault_after_event_us %0.2f", t_fault - 700.0);
        $display("METRIC first_fault_code %0d", first_fault_code);
        $display("METRIC il_peak_A %0.3f", il_max);
        check("fault latched within 40 us of the event", t_fault > 700.0 && (t_fault - 700.0) < 40.0, t_fault - 700.0);
        if (scen == "ocp")
          check("fault code is OCP (1)",                 first_fault_code == 3'd1, first_fault_code);
        else
          check("fault code is OCP (1) or UVP (3)",      first_fault_code == 3'd1 || first_fault_code == 3'd3, first_fault_code);
        check("peak inductor current < 1.35 x ILIM",   il_max < 1.35*ILIM, il_max);
        check("gates off after fault latched",         gate_act_after_fault == 0, gate_act_after_fault);
        check("restart after clear: regulating +/-1 %",(wf_avg > VREF_V*0.99) && (wf_avg < VREF_V*1.01), (wf_avg-VREF_V)/VREF_V*100.0);
        check("restart after clear: PGOOD",            pg_at_end, pg_at_end);
      end

      else if (scen == "ovp" || scen == "uvp") begin
        $display("METRIC t_fault_after_inject_us %0.2f", t_fault - 700.0);
        check("fault latched within 20 us of injection", t_fault > 700.0 && (t_fault - 700.0) < 20.0, t_fault - 700.0);
        check("correct fault code (OVP=2 / UVP=3)",      first_fault_code == ((scen == "ovp") ? 3'd2 : 3'd3), first_fault_code);
        check("gates off after fault latched",           gate_act_after_fault == 0, gate_act_after_fault);
        check("restart after clear: regulating +/-1 %",  (wf_avg > VREF_V*0.99) && (wf_avg < VREF_V*1.01), (wf_avg-VREF_V)/VREF_V*100.0);
        check("restart after clear: PGOOD",              pg_at_end, pg_at_end);
      end

      else if (scen == "uvlo") begin
        $display("METRIC t_idle_after_uvlo_us %0.2f", t_idle_dn - 700.0);
        check("shutdown to IDLE within 5 us of UVLO",   t_idle_dn > 700.0 && (t_idle_dn - 700.0) < 5.0, t_idle_dn - 700.0);
        check("no fault latched by UVLO",               t_fault < 0.0, t_fault);
        check("auto-restart: regulating +/-1 %",        (wf_avg > VREF_V*0.99) && (wf_avg < VREF_V*1.01), (wf_avg-VREF_V)/VREF_V*100.0);
        check("auto-restart: PGOOD",                    pg_at_end, pg_at_end);
      end

      if (n_fail == 0) $display("RESULT: PASS (%0d checks)", n_pass);
      else             $display("RESULT: FAIL (%0d of %0d checks failed)", n_fail, n_fail + n_pass);
      $fclose(f_wave); $fclose(f_edge); $fclose(f_zoom);
    end
  endtask
endmodule
