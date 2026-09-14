#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""Compare the RF link 2 h AFTER the RAK1906 install against the matched 2 h
window captured just before it.  The point is the FADE TAIL, not the mean: the
case+antenna bought most of its win in p5/worst, and those need hours to sample.
A 10-minute read flattered the tail by 14 dB last time - do not repeat that."""
import json, time, urllib.parse, urllib.request

PRE = {"mean": -77.97, "p5": -94.05, "worst": -101.0}   # 2 h window, 2026-09-13 11:03
WAIT = 7200

def q(e):
    u = "http://prometheus.example.lan:9090/api/v1/query?" + urllib.parse.urlencode({"query": e})
    try:
        r = json.load(urllib.request.urlopen(u, timeout=20))["data"]["result"]
        return float(r[0]["value"][1]) if r else None
    except Exception:
        return None

time.sleep(WAIT)
post = {"mean":  q("avg_over_time(davis_rssi_dbm[2h])"),
        "p5":    q("quantile_over_time(0.05, davis_rssi_dbm[2h])"),
        "worst": q("min_over_time(davis_rssi_dbm[2h])")}
slots, missed = q("davis_hop_slots_total"), q("davis_hop_missed_total")
resyncs = q("davis_hop_resyncs_total")

out = ["ANTENNA CHECK - 2 h after the RAK1906 install, matched windows:"]
worst_delta = 0.0
for k in ("mean", "p5", "worst"):
    a, b = PRE[k], post[k]
    if b is None:
        out.append(f"  {k:6s} pre {a:7.1f}   post   n/a"); continue
    d = b - a
    worst_delta = min(worst_delta, d)
    out.append(f"  {k:6s} pre {a:7.1f}   post {b:7.1f}   {d:+.1f} dB")
if slots:
    y = 100 * (slots - (missed or 0)) / slots
    out.append(f"  yield since power-up: {y:.2f}% ({int(slots)} slots, {int(missed or 0)} missed, "
               f"{int(resyncs or 0)} resyncs)")
out.append("  VERDICT: " + ("antenna looks undisturbed" if worst_delta > -4
                            else f"CHECK THE ANTENNA - lost {abs(worst_delta):.1f} dB somewhere"))
print("\n".join(out), flush=True)
