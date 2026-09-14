#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""Quiet guard for an overnight sweep run. SILENT unless something is wrong.

Not a heartbeat monitor - last night's watch reported every band change, which
was right for characterising the collapse but is noise now. Tonight the data is
what matters and it lands in Prometheus and sweep.jsonl regardless of whether
anyone is watching. The only thing worth waking up for is the run DYING.
"""
import json, time, urllib.parse, urllib.request

def q(e):
    u = "http://prometheus.example.lan:9090/api/v1/query?" + urllib.parse.urlencode({"query": e})
    try:
        r = json.load(urllib.request.urlopen(u, timeout=20))["data"]["result"]
        return float(r[0]["value"][1]) if r else None
    except Exception:
        return None                      # transient failure must not cry wolf

last_pass, stuck, fired = None, 0, set()
while True:
    time.sleep(600)
    up       = q('up{job="wxrx"}')
    tracking = q("davis_hop_tracking")
    resyncs  = q("davis_hop_resyncs_total")
    passes   = q("wxrx_sweep_pass")
    temp     = q("davis_temperature_fahrenheit")
    ctx = f"(temp {temp} F, pass {passes}, resyncs {resyncs})"

    if up == 0 and "down" not in fired:
        fired.add("down"); print(f"*** wxrx scrape is DOWN {ctx}", flush=True)
    if resyncs and resyncs > 0 and "resync" not in fired:
        fired.add("resync")
        print(f"*** LOCK LOST - resyncs={resyncs:.0f}, new behaviour, worth a look {ctx}", flush=True)

    # A sweep that stops advancing is the silent failure: the service looks
    # healthy, packets keep flowing, and the night's measurement is quietly gone.
    if passes is not None:
        if passes == last_pass:
            stuck += 1
            if stuck == 4 and "stuck" not in fired:      # ~40 min, a pass takes ~18
                fired.add("stuck")
                print(f"*** SWEEP STALLED - pass stuck at {passes:.0f} for ~40 min {ctx}", flush=True)
        else:
            stuck = 0
        last_pass = passes
