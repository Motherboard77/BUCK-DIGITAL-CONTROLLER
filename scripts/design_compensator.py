#!/usr/bin/env python3
"""Design the digital PID for the buck converter from a small-signal model.

Plant: duty (DPWM counts) -> ADC code, for a synchronous buck in CCM.
Loop : L(jw) = Gc(e^{jwTs}) * Gvd(jw) * exp(-jw*Td)
  Gc(z) = Kp + Ki/(1-z^-1) + Kd*(1-z^-1)          (same structure as rtl/pid_ctrl.sv)
  Td    = sample->duty-latch latency + modulator delay (trailing-edge DPWM ~ Ts/2)
"""
import numpy as np, json, sys

# ---- power stage / system parameters (must match model/buck_plant.sv) ----
VIN, L, RDCR, RDS, C, RESR = 5.0, 2.2e-6, 10e-3, 10e-3, 220e-6, 3e-3
R_LIGHT = 0.6            # 2 A at 1.2 V
R_HEAVY = 1/(1/0.6 + 1/1.2)   # 3 A
FCLK, FSW = 400e6, 500e3
N = int(FCLK/FSW)        # 800 counts/period
TS = 1/FSW
ADC_BITS, ADC_FS = 10, 2.0
LSB = ADC_FS/2**ADC_BITS
K_DC = (VIN/N)/LSB       # ADC LSB per DPWM count (3.2)
SAMPLE_CNT, ADC_LAT, PID_LAT = 752, 16, 3
TD = ((N - SAMPLE_CNT) + ADC_LAT*0 + 0)/FCLK + 0.5*TS    # sample -> next period start + modulator
FRAC = 12

def gvd(f, R):
    s = 2j*np.pi*f
    zc = RESR + 1/(s*C)
    zp = (R*zc)/(R + zc)
    return K_DC * zp/(zp + s*L + RDCR + RDS) * (RDS*0 + 1)  # (Vin/N)/LSB * Vo/Vsw

def gc(f, kp, ki, kd):
    z1 = np.exp(-2j*np.pi*f*TS)          # z^-1
    return kp + ki/(1 - z1) + kd*(1 - z1)

def loop(f, R, kp, ki, kd):
    return gc(f, kp, ki, kd) * gvd(f, R) * np.exp(-2j*np.pi*f*TD)

def margins(kp, ki, kd, R):
    f = np.logspace(2.5, np.log10(0.45*FSW), 4000)
    Lg = loop(f, R, kp, ki, kd)
    mag = np.abs(Lg); ph = np.unwrap(np.angle(Lg))
    # crossover = last downward crossing of 0 dB
    idx = np.where((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    if len(idx) == 0: return None
    i = idx[-1]
    fc = f[i]; pm = 180 + np.degrees(ph[i])
    # gain margin: where phase crosses -180 above fc
    pidx = np.where((ph[:-1] > -np.pi) & (ph[1:] <= -np.pi) & (f[:-1] > fc))[0]
    gm = np.inf
    if len(pidx): gm = -20*np.log10(mag[pidx[0]])
    return fc, pm, gm

# Chosen design (see docs/design_notes.md): grid search over (Kp, Ki, Kd) with
#   fc in 15..45 kHz, GM > 9 dB for light and heavy load, then Ki raised until the
#   slow integrator tail (tau ~ Ts*(1+K_DC*Kp)/(K_DC*Ki)) is ~25 us while keeping PM > 55 deg.
CHOSEN = dict(kp=1.0, ki=0.10, kd=8.0)

if __name__ == "__main__":
    kp, ki, kd = CHOSEN["kp"], CHOSEN["ki"], CHOSEN["kd"]
    q = lambda x: int(round(x*2**FRAC))
    print(f"K_DC = {K_DC:.3f} ADC LSB per DPWM count, Td = {TD*1e6:.2f} us, N = {N}")
    print(f"Kp={kp} Ki={ki} Kd={kd}  ->  Q.{FRAC}: KP={q(kp)} KI={q(ki)} KD={q(kd)}")
    print(f"slow integrator tail tau ~ {TS*(1+K_DC*kp)/(K_DC*ki)*1e6:.1f} us")
    for name, R in (("light 2 A", R_LIGHT), ("heavy 3 A", R_HEAVY), ("very light 1 A", 1.2)):
        fc, pm, gm = margins(kp, ki, kd, R)
        print(f"  {name:15s} R={R:.3f} ohm: fc={fc/1e3:.1f} kHz  PM={pm:.1f} deg  GM={gm:.1f} dB")
    json.dump(dict(kp=kp, ki=ki, kd=kd, KP=q(kp), KI=q(ki), KD=q(kd)), open("scripts/gains.json", "w"))
