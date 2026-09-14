#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""Turn davis-sweep's raw per-probe log into the drift-vs-temperature curve.

Run with no arguments the morning after a cold night:
    ssh wxrx.example.lan 'sudo -n cat /var/lib/wxrx/sweep.jsonl' > sweep.jsonl
    ./sweep-curve.py sweep.jsonl

WHY NOT JUST USE wxrx_sweep_best_abs_khz: the yield curve has a FLAT TOP - five
or so offsets all return 10/10 - so "best" is a tie broken arbitrarily by scan
order. It jumped -33/-35/-35 on 2026-09-13 while the real center moved smoothly.
The yield-weighted centroid and the full-yield midpoint are the honest estimators
and they agree with each other; trust those.
"""
import json, sys, statistics as st, datetime

path = sys.argv[1] if len(sys.argv) > 1 else "sweep.jsonl"
rows = [json.loads(l) for l in open(path) if l.strip()]
pts = [r for r in rows if not r.get("done")]
if not pts:
    sys.exit("no probe points in %s" % path)

passes = []
for p in sorted({r["pass"] for r in pts}):
    g = [r for r in pts if r["pass"] == p]
    if len(g) < 15:                      # partial pass at either end of the log
        continue
    tot = sum(r["good"] for r in g)
    if not tot:
        continue                          # a pass that heard nothing at all
    mx = max(r["good"] for r in g)
    full = [r["abs_khz"] for r in g if r["good"] >= mx]
    passes.append({
        "pass": p,
        "t": g[0]["t"],
        "outdoor": g[0].get("outdoor_f"),
        "rack": g[0].get("rack_f"),
        "centroid": sum(r["abs_khz"] * r["good"] for r in g) / tot,
        "midpoint": (min(full) + max(full)) / 2,
        "width": max(full) - min(full),
        "peak_yield": mx,
        "heard": tot,
    })

print(f"{len(passes)} usable passes from {path}\n")
print(f"{'time':>6} {'outdoor':>8} {'rack':>7} {'centroid':>9} {'midpoint':>9} {'width':>6} {'peak':>5}")
for r in passes:
    d = datetime.datetime.fromtimestamp(r["t"])
    print(f"{d:%H:%M} {str(r['outdoor']):>8} {str(r['rack']):>7} "
          f"{r['centroid']:9.2f} {r['midpoint']:9.1f} {r['width']:5.0f}k {r['peak_yield']:5d}")

def fit(xkey, label):
    xs = [r[xkey] for r in passes if r[xkey] is not None]
    ys = [r["centroid"] for r in passes if r[xkey] is not None]
    if len(xs) < 3 or len(set(xs)) < 2:
        print(f"\n{label}: not enough spread to fit")
        return
    mx, my = st.mean(xs), st.mean(ys)
    cov = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
    vx = sum((a - mx) ** 2 for a in xs)
    vy = sum((b - my) ** 2 for b in ys)
    slope = cov / vx
    r = cov / (vx * vy) ** 0.5 if vy else 0
    print(f"\n{label}: slope {slope:+.3f} kHz/F   r = {r:+.3f}   "
          f"over {min(xs):.1f}-{max(xs):.1f} F  (n={len(xs)})")

fit("outdoor", "center vs OUTDOOR temperature")
# The control variable. If this one also fits well, the receiver is moving too
# and the drift cannot be blamed on the transmitter alone.
fit("rack", "center vs RACK temperature  (should be FLAT - it is the control)")

print("\nReminders: daytime outdoor temp is NOT the transmitter's temperature "
      "(the ISS sits in the sun), so overnight passes are the clean ones.\n"
      "AT-cut crystals have a CUBIC tempco, so do not extrapolate a straight "
      "line into winter.")
