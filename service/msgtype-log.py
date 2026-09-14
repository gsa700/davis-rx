#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""Log raw Davis message-type payloads, to identify message 0x3.

0x3 is the one type the reverse-engineering community never pinned down. It is
NOT simply an absent sensor: 0x4 (UV) and 0x6 (solar radiation) are genuinely
not fitted here and sit permanently at the 10-bit maximum 1023, whereas 0x3 has
been seen at 1010.

Candidate explanations, distinguishable by SHAPE over a day rather than by
argument:
  - the ISS's internal regulated rail   -> flat, no day/night structure
  - the 18650 supply, sensed            -> slow drift with state of charge
  - an empty battery sense, floating    -> erratic, mostly pinned at 1023
  - not a voltage at all                -> none of the above

So we log 0x2 (supercap) and 0x7 (solar panel) alongside it as controls: both
track the sun hard, and whether 0x3 follows them or ignores them is the answer.
0x4 and 0x6 are logged too as known-absent references.

Polls the wxrx API rather than the serial port, so it cannot interfere with
reception and needs no change to wxrx itself.
"""
import json, os, time, urllib.request

API   = os.environ.get("MSGLOG_API", "http://127.0.0.1:8000/api/current.json")
OUT   = os.environ.get("MSGLOG_OUT", "/var/lib/wxrx/msgtypes.jsonl")
EVERY = float(os.environ.get("MSGLOG_INTERVAL", "10"))
WATCH = [int(x, 0) for x in os.environ.get("MSGLOG_TYPES", "2,3,4,6,7").split(",")]

# 10-bit sensor value: byte3 is the high 8 bits, the top 2 bits of byte4 the rest.
def value10(raw):
    b = bytes.fromhex(raw.replace(" ", ""))
    if len(b) < 5:
        return None
    return (b[3] << 2) | (b[4] >> 6)

last = {}
while True:
    try:
        with urllib.request.urlopen(API, timeout=8) as r:
            d = json.load(r)
        types = d.get("msgtypes") or {}
        now = time.time()
        for t in WATCH:
            e = types.get(str(t)) or types.get(t)
            if not e:
                continue
            seen = e.get("last_seen")
            if seen is None or last.get(t) == seen:
                continue          # same packet we already logged
            last[t] = seen
            raw = e.get("last_raw", "")
            rec = {
                "t": round(seen, 1),
                "type": t,
                "raw": raw,
                "v10": value10(raw),
                # context, so the shape can be read against the sun and the weather
                "outdoor_f": d.get("temperature_f"),
                "solar_v": d.get("solar_voltage_v"),
                "supercap_v": d.get("supercap_voltage_v"),
                "batt_low": d.get("battery_low"),
            }
            with open(OUT, "a") as f:
                f.write(json.dumps(rec) + "\n")
    except Exception:
        pass                       # a poll failure must never kill the logger
    time.sleep(EVERY)
