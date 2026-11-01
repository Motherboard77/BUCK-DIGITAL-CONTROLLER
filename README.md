# Digital Buck-Converter Controller — RTL + Mixed-Signal Verification

A synthesizable digital voltage-mode controller (SystemVerilog) for a synchronous buck converter,
verified closed-loop against a switching-level analog model, with startup, calibration, transient
and fault behaviour checked by a self-checking testbench, and the analog model cross-checked
against an independent SPICE (ngspice) simulation.

| | |
|---|---|
| Power stage | 5 V → 1.2 V, 2 A nominal (3 A step), L = 2.2 µH, C = 220 µF, fsw = 500 kHz |
| Controller clock | 400 MHz (800 counts / switching period), 10-bit ADC (2 V FS), one sample / period |
| Tools | Icarus Verilog 12, Verilator 5 (lint), Yosys (synthesis check), ngspice 42, Python (numpy/matplotlib) |

## Architecture

```
            +-------------------------- buck_ctrl.sv -----------------------------+
 vref ----->|  FSM: IDLE -> CAL -> SS -> REG        (FAULT latched, UVLO -> IDLE)   |
 en,vin_ok->|  CAL: ADC auto-zero, then pre-bias/discharge check                    |
            |  SS : reference ramp   REG: PGOOD    protections: OCP / OVP / UVP     |
 adc ------>|  err = ref - (adc - offset) --> pid_ctrl.sv --> duty (Q10.3)          |--> hs, ls
 ocp_comp ->|                                                    dpwm.sv: dead time, |
            |                                         cycle-by-cycle limit, dither   |
            +------------------------------------------------------------------------+
   model/afe_model.sv (ADC + comparator)  <---  model/buck_plant.sv (real-valued switching model)
```

* `rtl/dpwm.sv` – trailing-edge DPWM, dead time guaranteed by construction, cycle-by-cycle peak-current
  limit, 3-bit duty dithering (effective resolution finer than the clk/fsw ratio).
* `rtl/pid_ctrl.sv` – fixed-point PID `Kp + Ki/(1-z^-1) + Kd(1-z^-1)` (Q.12), clamped integrator with
  conditional-integration anti-windup.
* `rtl/buck_ctrl.sv` – power-controller FSM, calibration, soft-start, PGOOD, OCP/OVP/UVP, UVLO.
* `model/` – simulation-only: `real`-arithmetic power stage (HS/LS Rds(on), body diode in dead time, DCM,
  ESR, DCR) and an ADC/comparator model with injectable offset and noise.
* `scripts/design_compensator.py` – small-signal plant + loop model used to choose the PID gains.

## Results (all from `make run`; 8 scenarios, 51 self-checks, all passing)

| Metric | Result |
|---|---|
| Designed loop (model) | fc ≈ 19.7 kHz, PM 55–58° (54.7° at a 1 A load), GM ≈ 19 dB |
| ADC offset calibration | measured the injected +10 LSB offset exactly; DC error +0.04 % (vs. −1.6 % without calibration) |
| Startup | 200 µs soft-start, 0.32 % overshoot, peak inrush 3.7 A, PGOOD at ~364 µs |
| Output ripple (steady state) | 3.5 mV p-p, iL ripple 0.89 A p-p |
| Load step 2 → 3 A | droop 24.7 mV (2.1 %), back within ±1.5 % in 18 µs |
| Load release 3 → 2 A | overshoot 26 mV (2.2 %), back within ±1.5 % in 19 µs |
| Overload (10 A demand, 9 A limit) | iL held at 9.02 A, OCP latched after 26 µs, clean restart after clear |
| Hard short | iL peak 9.02 A, fault latched in 8 µs (UVP beats OCP, as expected), restart OK |
| OVP / UVP (sense-path injection) | latched in 4 µs / 8 µs; gates off; restart OK |
| UVLO | IDLE ~4 ns (1–2 clocks) after vin_ok falls; waits for output to discharge, then soft-starts |
| Shoot-through | 0 cycles in every scenario |
| Synthesis (Yosys generic) | 0 latches, 0 problems, 204 flip-flops, ~9.2k gates (see notes) |
| SPICE cross-check | Vo RMS difference 0.26 mV (max 0.78 mV) over 800 µs; iL ripple 0.857 A vs 0.856 A; droop 24.7 vs 24.1 mV |

Figures are in `docs/`: `startup_loadstep.png`, `load_transient.png`, `ripple_zoom.png`, `faults.png`,
`calibration.png`, `bode.png`, `spice_vs_sv.png`.

## Run it

```
make run        # compile + run all 8 scenarios, prints RESULT per scenario (logs in results/)
make lint       # Verilator lint of the RTL
make synth      # Yosys synthesis sanity check
make design     # print loop margins for the chosen PID gains
make plots      # regenerate docs/*.png
make spice      # replay RTL gate waveforms into ngspice and compare (needs `make run` first)
vvp -n build/sim.vvp +SCEN=main +ADC_OFF=25     # single scenario, custom injected ADC offset
```

## Things the simulations found and changed (worth knowing for interviews)

1. **Integrator too slow.** The first PID (Ki = 0.03) passed the load-step checks but the soft-start waveform
   showed an ~80 mV tracking error followed by a ~75 µs creep. Raising Ki to 0.10 cut the time constant to
   ~26 µs; phase margin dropped to the mid-50s of degrees.
2. **Test that did not test what it claimed.** The "OCP" scenario was really a short circuit, so UVP fired
   first. Split into an overload test (isolates OCP) and a short test.
3. **Calibration was measuring the output, not the ADC.** Restarting while the output was still discharging
   made residual output voltage look like ADC offset and the loop regulated 7 % high. Fixed with an ADC input
   auto-zero mux plus a separate discharge/pre-bias check before start.
4. **Synthesis bug invisible in simulation.** `if (!rst_n || clear)` inside an async-reset block simulates
   fine but is not a legal flop structure; Yosys rejected it. Fixed with a proper synchronous clear.

## Limitations (please read before presenting this as more than it is)

* **The SPICE comparison is an offline replay** (RTL gate waveforms → ngspice, no feedback inside ngspice).
  It validates the behavioral plant model; it is *not* a lock-step analog/digital co-simulation, and there is
  **no Verilog-A model** in this project. Lock-step co-simulation (e.g. ngspice XSPICE `d_cosim` with a
  Verilator model) is the natural next step.
* The analog side in the closed loop is a SystemVerilog `real` model (fixed 2.5 ns step), not `wreal`/UVM-AMS.
* Component values, targets and pass thresholds are my own representative choices, not from a customer spec.
* Only a resistive load, one input voltage (plus UVLO), nominal component values. No Monte-Carlo or
  temperature/tolerance sweeps, no input-voltage steps, no pre-biased soft-start (the controller waits for
  discharge instead).
* ADC auto-zero can only measure **positive** offsets (the ADC clamps at 0 codes).
* Loop design uses an averaged small-signal model with an approximate delay; crossover and phase margin were
  **not measured in simulation** (only checked through time-domain behaviour).
* Kd = 8 amplifies ADC quantization noise: steady-state duty hops by a few counts occasionally
  (visible in `startup_loadstep.png`); output ripple stays 3.5 mV p-p.
* UVP test = sense-path fault injection. Losing the sense path lets the output overshoot (~1.37 V, +14 %)
  for the few µs before UVP latches; a real design would add an independent hardware OVP.
* The three single-cycle 40-bit multipliers dominate area (~9.2k generic gates); a time-shared multiplier
  would shrink it a lot. Yosys numbers are generic-cell counts, not a timing-closed implementation.
* Nothing here has been run on an FPGA or in silicon.
