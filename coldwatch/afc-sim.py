#!/usr/bin/env python3
"""Simulate the davis-hop AFC loop against the MEASURED plateau.

Refitted 2026-09-16. The first model (flat to 4 kHz, then a gentle quadratic to
8) predicted that raising AFC_MARGIN to 3 would cut the dithering from 17 moves
per 40 cycles to 5. The real loop bang-banged between -27.0 and -28.5 on nearly
every cycle, so the model's shoulders were too soft. Refitted to the pass-37
sweep, where the shoulder is steep enough that a 0.75 kHz centring error already
produces a 3-packet imbalance across 6 probe slots.

Measured (pass 37, center -26.7 kHz, 10 slots per point):
    offset from center   -6.3   -4.3   -2.3   -0.3   +1.7   +3.7   +5.7
    good out of 10          2     10     10     10     10     10      5
"""
import random

AFC_PROBE_HZ       = 6000
AFC_SLOTS_PER_SIDE = 6
AFC_MARGIN         = 3
AFC_MIN_HZ, AFC_MAX_HZ = -60000, 10000

# Fitted to the measurements above: flat out to ~4.3 kHz, then a steep fall that
# reaches ~0.2 by 6.3 kHz and zero by ~7.5 kHz.
_PTS = [(0.0,1.0),(4.3,1.0),(5.3,0.62),(5.7,0.50),(6.3,0.20),(7.0,0.05),(7.5,0.0),(99,0.0)]
def p_good(delta_hz):
    d = abs(delta_hz)/1000.0
    for (x0,y0),(x1,y1) in zip(_PTS, _PTS[1:]):
        if d <= x1:
            return y0 + (y1-y0)*((d-x0)/(x1-x0)) if x1 > x0 else y0
    return 0.0

def probe(baseline, tx, n, rng):
    return sum(1 for _ in range(n) if rng.random() < p_good(baseline - tx))

def cycle(baseline, tx, rng, step_fn, silent=False):
    if silent:
        low = high = 0
    else:
        low  = probe(baseline - AFC_PROBE_HZ, tx, AFC_SLOTS_PER_SIDE, rng)
        high = probe(baseline + AFC_PROBE_HZ, tx, AFC_SLOTS_PER_SIDE, rng)
    d = high - low
    if low == 0 and high == 0:
        return baseline, "dead"
    if abs(d) < AFC_MARGIN:
        return baseline, "hold"
    step = step_fn(abs(d))
    baseline += step if d > 0 else -step
    return max(AFC_MIN_HZ, min(AFC_MAX_HZ, baseline)), ("up" if d > 0 else "down")

FIXED_1500 = lambda imb: 1500
FIXED_500  = lambda imb: 500
# Adaptive: a big imbalance means we are far out and should move fast; a small
# one means we are near the middle, where a coarse step is what makes it hunt.
ADAPTIVE   = lambda imb: 1500 if imb >= 5 else 500

def run(label, step_fn, baseline, tx_at, cycles, seed=7, silent=()):
    rng = random.Random(seed); errs=[]; acts=[]
    conv = None
    for i in range(cycles):
        tx = tx_at(i)
        baseline, act = cycle(baseline, tx, rng, step_fn, silent=(i in silent))
        errs.append(baseline-tx); acts.append(act)
        if conv is None and abs(baseline-tx) <= 1000: conv = i+1
    tail = errs[len(errs)//2:]
    rate = sum(p_good(e) for e in tail)/len(tail)
    moves = sum(1 for a in acts if a in ("up","down"))
    c = f"{conv} cyc ({conv*20} min)" if conv else "never"
    print(f"    {label:<22} converged {c:<18} worst-after {max(abs(e) for e in tail):5.0f} Hz"
          f"  moves {moves:2d}/{cycles}  decode {rate*100:5.1f}%")

print("\n=== 1. after a reflash: 6 kHz error to close (the convergence cost) ===")
for lbl,fn in (("step 1500 (current)",FIXED_1500),("step 500",FIXED_500),("adaptive 1500/500",ADAPTIVE)):
    run(lbl, fn, -33000, lambda i: -27000, 30)

print("\n=== 2. sitting centerd: how much does it fidget? (the reason to change) ===")
for lbl,fn in (("step 1500 (current)",FIXED_1500),("step 500",FIXED_500),("adaptive 1500/500",ADAPTIVE)):
    run(lbl, fn, -27000, lambda i: -27000, 40)

print("\n=== 3. overnight drift, 1.2 kHz over 8 h ===")
for lbl,fn in (("step 1500 (current)",FIXED_1500),("step 500",FIXED_500),("adaptive 1500/500",ADAPTIVE)):
    run(lbl, fn, -27000, lambda i: -27000 + int(1200*i/24), 24)

print("\n=== 4. aggressive drift, 6 kHz over 8 h (5x anything measured) ===")
for lbl,fn in (("step 1500 (current)",FIXED_1500),("step 500",FIXED_500),("adaptive 1500/500",ADAPTIVE)):
    run(lbl, fn, -30000, lambda i: -30000 + int(6000*i/24), 24)

print("\n=== 5. ISS silent 2 h: must hold ===")
for lbl,fn in (("adaptive 1500/500",ADAPTIVE),):
    run(lbl, fn, -27000, lambda i: -27000, 20, silent=range(5,11))
print()
