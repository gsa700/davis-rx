#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""rain-backfill — seed wxrx's rain state file from Prometheus history.

wxrx keeps rain totals and the tip-event log itself (the ISS only sends a
7-bit tip counter), so a fresh install starts with an empty 24 h rain graph.
If Prometheus has been scraping `davis_rain_counter` from an earlier wxrx
instance, this rebuilds the state file from that history using the same
unwrap rules as wxrx.py: delta mod 128 per sample, a jump of 64 or more is a
counter reset and is skipped.

Prometheus is queried one day at a time at the scrape step, because a 7-day
range at 15 s trips the 11 000-point cap and returns HTTP 400.

Run with wxrx STOPPED, then start it:

    sudo systemctl stop wxrx
    sudo python3 rain-backfill.py --prom http://prometheus.example.lan:9090 \
        --out /var/lib/wxrx/state.json
    sudo chown wxrx:dialout /var/lib/wxrx/state.json
    sudo systemctl start wxrx
"""
import argparse, json, os, sys, time, urllib.parse, urllib.request

RAIN_LOG_DAYS = 7      # keep in step with wxrx.py

def query_range(prom, metric, start, end, step):
    q = urllib.parse.urlencode({"query": metric, "start": start, "end": end, "step": step})
    with urllib.request.urlopen(f"{prom}/api/v1/query_range?{q}", timeout=30) as r:
        res = json.load(r)["data"]["result"]
    out = []
    for series in res:                       # normally exactly one
        out.extend((float(t), int(float(v))) for t, v in series["values"])
    return sorted(out)

def unwrap(samples):
    """Fold (epoch, counter) samples into wxrx's rain dict. Mirrors note_rain_counter()."""
    rain = {"tips_total": 0, "last_counter": None, "last_counter_epoch": None,
            "day": None, "today_tips": 0, "last_tip_epoch": None, "events": []}
    for t, c in samples:
        day = time.strftime("%Y-%m-%d", time.localtime(t))
        if rain["day"] != day:
            rain["day"], rain["today_tips"] = day, 0
        if rain["last_counter"] is not None:
            delta = (c - rain["last_counter"]) % 128
            if 0 < delta < 64:
                rain["tips_total"] += delta
                rain["today_tips"] += delta
                rain["last_tip_epoch"] = t
                rain["events"].append([round(t, 1), delta])
        rain["last_counter"], rain["last_counter_epoch"] = c, t
    return rain

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--prom", required=True, help="Prometheus base URL, e.g. http://prometheus.example.lan:9090")
    ap.add_argument("--out", default="/var/lib/wxrx/state.json", help="state file to write")
    ap.add_argument("--days", type=int, default=RAIN_LOG_DAYS, help="days of history to pull")
    ap.add_argument("--step", default="15s", help="query step; use your scrape interval")
    ap.add_argument("--metric", default="davis_rain_counter")
    ap.add_argument("--dry-run", action="store_true", help="print the result instead of writing it")
    a = ap.parse_args()

    if os.path.exists(a.out) and not a.dry_run:
        sys.exit(f"rain-backfill: {a.out} already exists; remove it first if you really want to reseed")

    now = time.time()
    samples = []
    for d in range(a.days, 0, -1):
        start, end = now - d * 86400, now - (d - 1) * 86400
        chunk = query_range(a.prom, a.metric, start, end, a.step)
        print(f"rain-backfill: {time.strftime('%Y-%m-%d', time.localtime(start))}: {len(chunk)} samples",
              file=sys.stderr)
        samples.extend(chunk)
    if not samples:
        sys.exit("rain-backfill: no samples returned; check --prom and that the metric exists")

    rain = unwrap(samples)
    # Do not hand wxrx a stale counter baseline: it re-baselines anything older
    # than 2 h itself, but a live install will see a fresh reading within seconds.
    rain["last_counter"] = None
    print(f"rain-backfill: {rain['tips_total']} tips in {len(rain['events'])} bursts over "
          f"{a.days} days, {rain['today_tips']} today", file=sys.stderr)

    if a.dry_run:
        json.dump(rain, sys.stdout, indent=1); print()
        return
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    tmp = a.out + ".tmp"
    with open(tmp, "w") as f:
        json.dump(rain, f)
    os.replace(tmp, a.out)
    print(f"rain-backfill: wrote {a.out}", file=sys.stderr)

if __name__ == "__main__":
    main()
