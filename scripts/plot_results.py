#!/usr/bin/env python3
"""Generate docs/*.png from results/*.csv (run `make run` first)."""
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import scripts.design_compensator as d

STATE = {0: "IDLE", 1: "CAL", 2: "SS", 3: "REG", 4: "FAULT"}
def load(n): return pd.read_csv(f"results/{n}.csv")

# ---------------- 1. startup + load transient ----------------
m = load("main")
fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
ax[0].plot(m.t_us, m.vo, label="Vo"); ax[0].plot(m.t_us, m.ref_v, "--", label="reference (soft-start ramp)")
ax[0].set(ylabel="V"); ax[0].legend(loc="lower right"); ax[0].set_title("Closed-loop run: calibration → soft-start → regulation → load step (2→3 A) → release")
ax[1].plot(m.t_us, m.il); ax[1].set(ylabel="iL [A]")
ax[2].plot(m.t_us, m.duty_counts/800*100); ax[2].set(ylabel="duty [%]")
ax[3].step(m.t_us, m.state, where="post", label="state"); ax[3].plot(m.t_us, m.pgood*0.5 - 1, label="PGOOD (offset)")
ax[3].set(ylabel="FSM state", xlabel="t [µs]", yticks=[0, 1, 2, 3, 4], yticklabels=[STATE[i] for i in range(5)]); ax[3].legend(loc="center right")
for a in ax:
    a.grid(alpha=.3)
    for t in (600, 900): a.axvline(t, color="gray", lw=.6, ls=":")
plt.tight_layout(); plt.savefig("docs/startup_loadstep.png", dpi=130); plt.close()

# ---------------- 2. load-step zoom ----------------
fig, ax = plt.subplots(1, 2, figsize=(12, 4))
for a, (t0, t1, ttl) in zip(ax, [(590, 760, "Load step 2 A → 3 A at 600 µs"), (890, 1060, "Load release 3 A → 2 A at 900 µs")]):
    w = m[(m.t_us >= t0) & (m.t_us < t1)]
    a.plot(w.t_us, w.vo*1e3); a.axhline(1200*1.015, color="r", lw=.6, ls=":"); a.axhline(1200*.985, color="r", lw=.6, ls=":")
    a.set(title=ttl + "  (dotted: ±1.5 %)", xlabel="t [µs]", ylabel="Vo [mV]"); a.grid(alpha=.3)
plt.tight_layout(); plt.savefig("docs/load_transient.png", dpi=130); plt.close()

# ---------------- 3. steady-state ripple ----------------
z = pd.read_csv("results/main_zoom.csv")
fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
ax[0].plot(z.t_us, z.vo*1e3); ax[0].set(ylabel="Vo [mV]", title="Steady state, 2 switching periods (500 kHz)")
ax[1].plot(z.t_us, z.il); ax[1].set(ylabel="iL [A]")
ax[2].plot(z.t_us, z.hs, label="HS"); ax[2].plot(z.t_us, z.ls*0.9, label="LS"); ax[2].set(ylabel="gates", xlabel="t [µs]"); ax[2].legend()
for a in ax: a.grid(alpha=.3)
plt.tight_layout(); plt.savefig("docs/ripple_zoom.png", dpi=130); plt.close()

# ---------------- 4. faults ----------------
cases = [("ocp", "Overload 10 A > 9 A limit @700 µs → OCP latch; clear @950 µs → restart"),
         ("short", "Hard short @700 µs → fast latched fault; clear @950 µs → restart"),
         ("ovp", "Sense-path reads +400 codes @700 µs → OVP; clear @900 µs → restart"),
         ("uvp", "Sense-path reads −400 codes @700 µs → UVP; clear @900 µs → restart"),
         ("uvlo", "Vin UVLO @700 µs, returns @800 µs → wait for discharge → soft-start")]
fig, ax = plt.subplots(len(cases), 1, figsize=(11, 3*len(cases)))
for a, (n, ttl) in zip(ax, cases):
    w = load(n)
    a.plot(w.t_us, w.vo, label="Vo [V]"); a.plot(w.t_us, w.il/5, label="iL / 5 [A/5]", lw=.8, alpha=.8)
    a2 = a.twinx(); a2.step(w.t_us, w.state, where="post", color="tab:red", lw=1, label="state")
    a2.set_yticks([0, 1, 2, 3, 4]); a2.set_yticklabels([STATE[i] for i in range(5)], fontsize=7)
    a.set_title(ttl, fontsize=9); a.grid(alpha=.3); a.set_xlim(500, 1700 if n == "uvlo" else 1450)
    if n == "ocp": a.legend(loc="upper left", fontsize=7)
plt.tight_layout(); plt.savefig("docs/faults.png", dpi=120); plt.close()

# ---------------- 5. calibration effect ----------------
nc = load("nocal")
fig, ax = plt.subplots(figsize=(9, 4))
for df, lbl in ((m, "calibrated (auto-zero)"), (nc, "calibration bypassed")):
    w = df[(df.t_us > 450) & (df.t_us < 600)]
    ax.plot(w.t_us, w.vo*1e3, label=f"{lbl}: mean {w.vo.mean()*1e3:.1f} mV")
ax.axhline(1200, color="k", lw=.6, ls=":"); ax.set(xlabel="t [µs]", ylabel="Vo [mV]", title="Effect of ADC-offset calibration (injected offset +10 LSB = 19.5 mV)")
ax.legend(); ax.grid(alpha=.3); plt.tight_layout(); plt.savefig("docs/calibration.png", dpi=130); plt.close()

# ---------------- 6. Bode ----------------
f = np.logspace(2.5, np.log10(0.45*d.FSW), 1500)
kp, ki, kd = d.CHOSEN["kp"], d.CHOSEN["ki"], d.CHOSEN["kd"]
fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
for R, lbl in ((d.R_LIGHT, "2 A load"), (d.R_HEAVY, "3 A load")):
    Lg = d.loop(f, R, kp, ki, kd); fc, pm, gm = d.margins(kp, ki, kd, R)
    ax[0].semilogx(f/1e3, 20*np.log10(np.abs(Lg)), label=f"{lbl}: fc={fc/1e3:.1f} kHz, PM={pm:.0f}°, GM={gm:.0f} dB")
    ax[1].semilogx(f/1e3, np.degrees(np.unwrap(np.angle(Lg))))
ax[0].axhline(0, color="k", lw=.6); ax[0].set(ylabel="|L| [dB]", title="Designed loop gain L(s) = Gc·Gvd·delay"); ax[0].legend(); ax[0].grid(alpha=.3, which="both")
ax[1].axhline(-180, color="k", lw=.6); ax[1].set(ylabel="phase [deg]", xlabel="f [kHz]"); ax[1].grid(alpha=.3, which="both")
plt.tight_layout(); plt.savefig("docs/bode.png", dpi=130); plt.close()
print("figures written to docs/")
