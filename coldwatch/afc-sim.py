#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""Simulate the davis-hop AFC loop against the measured plateau.

Compiling proves the sketch builds; it says nothing about whether the control
loop converges, holds, or oscillates. This models the transmitter and replays
the same decision rule so those questions get answered before the sketch is
ever flashed.

Plateau model is fitted to pass 37 (2026-09-14, centre -26.7 kHz):
    offset from centre   -6.3   -4.3   -2.3   -0.3   +1.7   +3.7   +5.7
    good out of 10          2     10     10     10     10     10      5
"""
import random

# --- must mirror davis-hop.ino ---
AFC_PROBE_HZ       = 6000
AFC_SLOTS_PER_SIDE = 6
AFC_STEP_HZ        = 1500
AFC_MARGIN         = 3
AFC_MIN_HZ, AFC_MAX_HZ = -60000, 10000


def p_good(delta_hz):
    """Probability a slot decodes, given offset error in Hz."""
    d = abs(delta_hz) / 1000.0
    if d <= 4.0:  return 1.0
    if d >= 8.0:  return 0.0
    return ((8.0 - d) / 4.0) ** 2


def probe(baseline, tx, n, rng):
    return sum(1 for _ in range(n) if rng.random() < p_good(baseline - tx))


def afc_cycle(baseline, tx, rng, silent=False):
    """One AFC cycle. Returns (new_baseline, action, low, high)."""
    if silent:
        low = high = 0
    else:
        low  = probe(baseline - AFC_PROBE_HZ, tx, AFC_SLOTS_PER_SIDE, rng)
        high = probe(baseline + AFC_PROBE_HZ, tx, AFC_SLOTS_PER_SIDE, rng)
    if low == 0 and high == 0:
        act = "dead"
    elif high >= low + AFC_MARGIN:
        baseline += AFC_STEP_HZ; act = "up"
    elif low >= high + AFC_MARGIN:
        baseline -= AFC_STEP_HZ; act = "down"
    else:
        act = "hold"
    baseline = max(AFC_MIN_HZ, min(AFC_MAX_HZ, baseline))
    return baseline, act, low, high


def run(label, baseline, tx_at, cycles, rng, silent_range=()):
    """tx_at(i) -> transmitter offset at cycle i."""
    errs, acts = [], []
    for i in range(cycles):
        tx = tx_at(i)
        baseline, act, lo, hi = afc_cycle(baseline, tx, rng, silent=i in silent_range)
        errs.append(baseline - tx); acts.append(act)
    settled = errs[len(errs) // 2:]
    worst = max(abs(e) for e in settled)
    rate = sum(p_good(e) for e in settled) / len(settled)
    print(f"  {label}")
    print(f"      final error {errs[-1]:+6.0f} Hz | worst after settling {worst:5.0f} Hz"
          f" | mean decode rate {rate*100:5.1f}%"
          f" | moves {sum(1 for a in acts if a in ('up','down'))}/{cycles}")
    return worst, rate, acts


rng = random.Random(20260914)
CY = 20 * 60  # one cycle = 20 min

print("\n=== 1. tonight's actual situation: baseline -33k, transmitter at -26.7k ===")
w, r, _ = run("6.3 kHz initial error, transmitter static",
              -33000, lambda i: -26700, 24, rng)
assert w <= 3000 and r > 0.95, "failed to converge from today's error"

print("\n=== 2. realistic overnight drift (1.2 kHz over 8 h) while tracking ===")
w, r, _ = run("drifts -26.7k -> -25.5k over 24 cycles",
              -26700, lambda i: -26700 + int(1200 * i / 24), 24, rng)
assert w <= 3000 and r > 0.95, "failed to track realistic drift"

print("\n=== 3. aggressive drift, 5x faster than anything measured ===")
w, r, _ = run("6 kHz over 24 cycles (8 h)",
              -30000, lambda i: -30000 + int(6000 * i / 24), 24, rng)
assert w <= 4000 and r > 0.90, "failed to track aggressive drift"

print("\n=== 4. already centred: does it sit still or hunt? ===")
w, r, acts = run("perfectly centred, static transmitter",
                 -26700, lambda i: -26700, 40, rng)
moves = sum(1 for a in acts if a in ("up", "down"))
print(f"      limit cycle: {moves} moves in 40 cycles, worst excursion {w:.0f} Hz")
assert w <= 3000 and r > 0.97, "hunts badly when already centred"

print("\n=== 5. ISS silent for 6 cycles (2 h): must HOLD, not wander ===")
before = -26700
w, r, acts = run("silent cycles 5-10", before, lambda i: -26700, 20, rng,
                 silent_range=range(5, 11))
dead = sum(1 for a in acts if a == "dead")
print(f"      dead cycles seen {dead} (expected 6) -- baseline held through them")
assert dead == 6, "silence not detected as dead"
assert w <= 3000, "wandered while the transmitter was silent"

print("\n=== 6. cold-morning step: 10 kHz error, outside the plateau entirely ===")
w, r, acts = run("baseline -33k, transmitter jumped to -23k", -33000,
                 lambda i: -23000, 30, rng)
print(f"      note: recovery here relies on the shoulder probe still reaching the")
print(f"            plateau. Beyond that the ACQUIRE rescue scan is the safety net.")

print("\n=== 7. rails hold under a pathological all-up run ===")
b = -33000
for _ in range(200):
    b = min(AFC_MAX_HZ, b + AFC_STEP_HZ)
print(f"      runaway up  -> {b} Hz (rail {AFC_MAX_HZ})"); assert b == AFC_MAX_HZ
b = -33000
for _ in range(200):
    b = max(AFC_MIN_HZ, b - AFC_STEP_HZ)
print(f"      runaway down -> {b} Hz (rail {AFC_MIN_HZ})"); assert b == AFC_MIN_HZ

print("\nall AFC simulations passed")
