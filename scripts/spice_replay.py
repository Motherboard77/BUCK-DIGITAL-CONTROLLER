#!/usr/bin/env python3
"""Replay the RTL's gate-drive waveforms (results/main_edges.csv) into an ngspice
switching netlist of the same power stage and compare with the SystemVerilog
`real` plant model (results/main.csv, results/main_zoom.csv).

This is an OFFLINE replay (RTL -> SPICE, no feedback inside ngspice), i.e. a model
cross-check, not a lock-step co-simulation.  Output: spice/ files, results/spice_compare.json,
docs/spice_vs_sv.png
"""
import subprocess, json, sys
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

T_END_US = 800.0           # startup + 2 A -> 3 A load step at 600 us
TR = 1e-9                  # gate edge time
edges = pd.read_csv("results/main_edges.csv")
edges = edges[edges.t_ns <= T_END_US*1e3 + 10]

def pwl(name, node, t_ns, vals, vhigh=5.0):
    pts = [(0.0, vals[0]*vhigh)]
    prev = vals[0]
    for t, v in zip(t_ns[1:], vals[1:]):
        if v != prev:
            ts = t*1e-9
            pts.append((ts - TR, prev*vhigh)); pts.append((ts, v*vhigh)); prev = v
    body = "\n".join(f"+ {t:.12g} {v:g}" for t, v in pts)
    return f"{name} {node} 0 PWL(\n{body}\n+ )\n"

t = edges.t_ns.values
inc  = pwl("Vhs", "hsg", t, edges.hs.values)
inc += pwl("Vls", "lsg", t, edges.ls.values)
inc += pwl("Vld", "ldg", t, (edges.rload.values < 0.5).astype(float))   # 1 = heavy-load switch closed
open("spice/gates.inc", "w").write(inc)

net = f"""* Synchronous buck power stage - RTL gate waveforms replayed (see scripts/spice_replay.py)
Vin   vin 0 DC 5
.include gates.inc
S1    vin sw hsg 0 SWM
S2    sw  0  lsg 0 SWM
D1    sw  vin DB
D2    0   sw  DB
Lind  sw  n1 2.2u
Rdcr  n1  lo 10m
Cout  lo  cx 220u
Resr  cx  0  3m
Rl    lo  0  0.6
S3    lo  r2 ldg 0 SWM
R2    r2  0  1.2
.model SWM SW(Vt=2.5 Vh=0.1 Ron=10m Roff=1Meg)
.model DB  D(IS=2.5e-10 N=1)
.options reltol=1e-4 abstol=1e-9 vntol=1e-7 method=gear
.tran 2n {T_END_US}u 0 2n
.control
set noaskquit
run
wrdata spice_out.txt v(lo) i(Lind)
quit
.endc
.end
"""
open("spice/buck_replay.cir", "w").write(net)

print("running ngspice ...", flush=True)
r = subprocess.run(["ngspice", "-b", "buck_replay.cir"], cwd="spice", capture_output=True, text=True, timeout=1500)
if r.returncode != 0 or not __import__("os").path.exists("spice/spice_out.txt"):
    print(r.stdout[-2000:], r.stderr[-2000:]); sys.exit(1)

sp = np.loadtxt("spice/spice_out.txt")           # cols: t, v(lo), t, i(Lind)
ts, vs, il_s = sp[:, 0]*1e6, sp[:, 1], sp[:, 3]

sv = pd.read_csv("results/main.csv"); sv = sv[sv.t_us <= T_END_US]
vs_i = np.interp(sv.t_us, ts, vs); il_i = np.interp(sv.t_us, ts, il_s)
dv = (vs_i - sv.vo.values)*1e3

def win(a, b, x, tt): m = (tt >= a) & (tt < b); return x[m]
res = {}
res["vo_rms_err_mV_full"]   = float(np.sqrt(np.mean(dv**2)))
res["vo_max_abs_err_mV"]    = float(np.max(np.abs(dv)))
m = (sv.t_us >= 550) & (sv.t_us < 600)
res["vo_avg_spice_V_550_600"] = float(np.mean(vs_i[m]));   res["vo_avg_sv_V_550_600"] = float(np.mean(sv.vo.values[m]))
m2 = (sv.t_us >= 700) & (sv.t_us < 800)
res["vo_avg_spice_V_700_800"] = float(np.mean(vs_i[m2]));  res["vo_avg_sv_V_700_800"] = float(np.mean(sv.vo.values[m2]))
mm = (sv.t_us >= 600) & (sv.t_us < 700)
res["droop_spice_mV"] = float((1.2 - vs_i[mm].min())*1e3); res["droop_sv_mV"] = float((1.2 - sv.vo.values[mm].min())*1e3)

# ripple in the zoom window (fine resolution)
z = pd.read_csv("results/main_zoom.csv")
zs_v = np.interp(z.t_us, ts, vs); zs_i = np.interp(z.t_us, ts, il_s)
res["il_ripple_pp_A_sv"] = float(z.il.max() - z.il.min()); res["il_ripple_pp_A_spice"] = float(zs_i.max() - zs_i.min())
res["il_avg_A_sv"] = float(z.il.mean());                   res["il_avg_A_spice"] = float(zs_i.mean())
res["il_zoom_rms_err_A"] = float(np.sqrt(np.mean((zs_i - z.il.values)**2)))
json.dump(res, open("results/spice_compare.json", "w"), indent=2)
for k, v in res.items(): print(f"{k:28s} {v:10.4f}")

# ---------------- plot ----------------
fig, ax = plt.subplots(2, 2, figsize=(13, 8))
ax[0,0].plot(sv.t_us, sv.vo, label="SV real model (RTL in the loop)", lw=1.6)
ax[0,0].plot(ts[ts <= T_END_US], vs[ts <= T_END_US], "--", label="ngspice (RTL gate replay)", lw=1.2)
ax[0,0].set(title="Output voltage: startup + 2 A→3 A load step", xlabel="t [µs]", ylabel="Vo [V]"); ax[0,0].legend(); ax[0,0].grid(alpha=.3)
ax[0,1].plot(sv.t_us, dv, lw=.8); ax[0,1].set(title="Vo difference (SPICE − SV)", xlabel="t [µs]", ylabel="mV"); ax[0,1].grid(alpha=.3)
mz = (sv.t_us > 590) & (sv.t_us < 760)
ax[1,0].plot(sv.t_us[mz], sv.vo[mz], label="SV"); ax[1,0].plot(ts[(ts>590)&(ts<760)], vs[(ts>590)&(ts<760)], "--", label="ngspice")
ax[1,0].set(title="Load-step zoom", xlabel="t [µs]", ylabel="Vo [V]"); ax[1,0].legend(); ax[1,0].grid(alpha=.3)
ax[1,1].plot(z.t_us, z.il, label="SV"); ax[1,1].plot(z.t_us, zs_i, "--", label="ngspice")
ax[1,1].set(title="Inductor current, steady state (2 switching periods)", xlabel="t [µs]", ylabel="iL [A]"); ax[1,1].legend(); ax[1,1].grid(alpha=.3)
plt.tight_layout(); plt.savefig("docs/spice_vs_sv.png", dpi=130)
print("wrote docs/spice_vs_sv.png")
