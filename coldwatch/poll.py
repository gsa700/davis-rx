#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""Poll Prometheus for Davis RX health during a cold night. One CSV row per poll;
stdout only on band changes, 2 h heartbeats, or recovery/alarm."""
import json, sys, time, urllib.parse, urllib.request, datetime, os

PROM = "http://prometheus.example.lan:9090/api/v1/query"
CSV = os.path.expanduser("~/Projects/davis-rx/coldwatch/cold-night-2026-09-13.csv")
Q = {
    "ok":      "rate(davis_packets_total_ok[10m])*60",
    "bad":     "rate(davis_packets_total_bad[10m])*60",
    "missed":  "rate(davis_hop_missed_total[10m])*60",
    "temp":    "davis_temperature_fahrenheit",
    "rssi":    "avg_over_time(davis_rssi_dbm[10m])",
    "tracking":"davis_hop_tracking",
    "resyncs": "davis_hop_resyncs_total",
    "age":     "davis_packet_age_seconds",
}

def q(expr):
    try:
        u = PROM + "?" + urllib.parse.urlencode({"query": expr})
        r = json.load(urllib.request.urlopen(u, timeout=15))["data"]["result"]
        return float(r[0]["value"][1]) if r else None
    except Exception:
        return None            # a transient failure must not kill the monitor

def band(ok):
    if ok is None:  return "NODATA"
    if ok >= 22:    return "NORMAL"
    if ok >= 18:    return "SLIGHT"
    if ok >= 12:    return "DEGRADED"
    if ok >= 5:     return "BAD"
    return "CRITICAL"

if not os.path.exists(CSV):
    with open(CSV, "w") as f:
        f.write("time,ok_per_min,bad_per_min,missed_per_min,temp_f,rssi_dbm,tracking,resyncs,age_s\n")

prev_band, last_beat, good_run = None, 0.0, 0
while True:
    v = {k: q(e) for k, e in Q.items()}
    now = datetime.datetime.now()
    with open(CSV, "a") as f:
        f.write(now.strftime("%Y-%m-%d %H:%M") + "," +
                ",".join("" if v[k] is None else f"{v[k]:.2f}" for k in
                         ("ok","bad","missed","temp","rssi","tracking","resyncs","age")) + "\n")

    ok, b = v["ok"], band(v["ok"])
    line = (f"{now:%H:%M}  {ok if ok is None else round(ok,1)} pkt/min  "
            f"[{b}]  temp {v['temp']} F  rssi {None if v['rssi'] is None else round(v['rssi'],1)}  "
            f"bad {None if v['bad'] is None else round(v['bad'],1)}/min  "
            f"tracking={v['tracking']}  resyncs={v['resyncs']}")

    emit = None
    if b != prev_band and prev_band is not None:
        emit = f"BAND {prev_band} -> {b}   {line}"
    elif time.time() - last_beat > 7200:
        emit = f"heartbeat  {line}"
    if v["tracking"] == 0 or (v["resyncs"] or 0) > 0:
        emit = f"*** LOCK PROBLEM (tracking/resyncs changed)  {line}"

    good_run = good_run + 1 if (ok is not None and ok >= 22) else 0
    if good_run >= 3 and prev_band not in (None, "NORMAL"):
        print(f"RECOVERED as predicted: {line}", flush=True)
        print(f"Full curve: {CSV}", flush=True)
        sys.exit(0)

    if emit:
        print(emit, flush=True)
        last_beat = time.time()
    prev_band = b
    time.sleep(600)
