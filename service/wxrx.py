#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""
wxrx — Davis Vantage Pro II ISS receiver service.

Reads CRC-validated packets from a RAK4631 over USB serial, decodes them, and
serves the result three ways from one process:

    /metrics           Prometheus exposition  -> Grafana
    /api/current.json  JSON snapshot          -> desktop app / web page
    /                  tiny HTML page         -> a browser window you leave open

The RAK does demodulation and CRC; this only parses and presents.  Keeping that
boundary means the decode logic can change without touching any consumer.

Serial line format produced by davis-rx.ino / davis-hop.ino:
    [CRC OK ] 88 02 64 33 CB 0E B9 57 FF FF  msg=0x8 id=0 rssi=-79 ch=24 [direct]
Bytes are already bit-reversed by the sketch (Davis transmits LSB-first).
The `ch=` field is appended by the hop follower (milestone 2); LINE's regex
stops at rssi=, so both firmwares parse with the same pattern.

The hop follower also emits periodic STATUS lines, which carry the lock state:
    STATUS state=TRACK ch=24 heard=91 good=88 bad=3 slots=94 missed=6 streak=0 resyncs=1
These are parsed separately (STATUS_KV) into davis_hop_* metrics.  Without them
a hop failure is invisible: the receiver would fall back to hearing 1 packet in
51 and simply look like a weak signal rather than a lost lock.
"""
import json, os, re, threading, time, http.server, socketserver, sys

PORT   = int(os.environ.get("WXRX_PORT", "8000"))
# Rain bookkeeping is persisted so a service restart does not zero "today" or
# the tip log behind the 24 h graph. With ProtectSystem=strict the unit grants
# this path via StateDirectory=wxrx. Override for a dev run on a workstation.
STATE_FILE   = os.environ.get("WXRX_STATE", "/var/lib/wxrx/state.json")
# davis-sweep's per-offset results. One JSON object per line, appended forever -
# a sweep pass is ~21 lines every ~18 min, so this stays small, and the whole
# point is to accumulate the drift-vs-temperature curve across many cold nights.
SWEEP_LOG    = os.environ.get("WXRX_SWEEP_LOG", "/var/lib/wxrx/sweep.jsonl")
TIP_INCHES   = float(os.environ.get("WXRX_TIP_INCHES", "0.01"))   # US bucket; 0.2 mm metric
RAIN_LOG_DAYS = 7
# 24 h graphs for everything except rain come from Prometheus, which already
# scrapes this service; the page asks us and we proxy so the browser never
# needs to know where Prometheus lives. Unset = graphs show "no history".
PROM_URL = os.environ.get("WXRX_PROM_URL", "").rstrip("/")
# Use the stable by-id symlink, NOT /dev/ttyACMn. The number is assigned in
# enumeration order, so unplugging and replugging can move the RAK from ACM0 to
# ACM1 and strand a hardcoded path forever. by-id follows the serial number.
# Overridable so a SECOND instance can run against a different receiver — the
# RFM69 Feather alongside the RAK, same code, separate port and state dir.
DEVICE = os.environ.get(
    "WXRX_DEVICE",
    "/dev/serial/by-id/usb-RAKwireless_WisBlock_RAK4631_C192A44D02C1376F-if00")
BAUD   = 115200

LINE = re.compile(
    r"\[CRC (OK|BAD)\s*\]\s+((?:[0-9A-Fa-f]{2}\s+){10})msg=0x([0-9A-Fa-f])\s+id=(\d+)\s+rssi=(-?\d+)")
# STATUS lines are plain key=value; parse generically so adding a field to the
# firmware never requires a matching change here.
STATUS_KV = re.compile(r"([A-Za-z_][A-Za-z0-9_-]*)=(-?[\w.]+)")

state = {
    "wind_speed_mph": None, "wind_direction_deg": None,
    "temperature_f": None, "humidity_pct": None,
    "wind_gust_mph": None, "rain_counter": None,
    "rain_rate_inph": None, "rain_tbt_s": None,
    "rssi_dbm": None, "station_id": None, "battery_low": None,
    # ISS power telemetry (msg 0x7 / 0x2). Scaling per weewx-meteostick:
    # raw 10-bit = (b3<<2)|(b4>>6), 0x3FF = not available, volts = raw/300.
    "solar_voltage_v": None, "solar_raw": None, "supercap_voltage_v": None,
    "last_packet_epoch": None, "last_msg_type": None,
    "packets_ok": 0, "packets_bad": 0,
    "last_raw": None,
    # hop-follower (milestone 2) telemetry; stay None on milestone-1 firmware
    # per-message-type inventory: what the ISS actually sends, decoded or not.
    # keyed by int msgtype -> {count, last_raw, last_seen, bytes}
    "msgtypes": {},
    # RAK1906 (BME680) fitted to the RAK 2026-09-13. RACK-ambient temp/hum (the
    # case sits on top of the rack, dead centre, and the rack has a thermostatic
    # fan — so this is a rack thermal sensor, NOT room temperature and NOT case
    # self-heating) + the station pressure we actually wanted it for.
    # Stay None on firmware without the sensor line.
    "env_temp_c": None, "env_humidity_pct": None, "env_pressure_hpa": None,
    "env_epoch": None,
    # davis-sweep telemetry; stays None on davis-hop, which is the honest signal
    # that no sweep is running rather than a misleading zero.
    "sweep_pass": None, "sweep_best_khz": None, "sweep_best_abs_khz": None,
    "sweep_best_good": None, "sweep_pegged": None, "sweep_epoch": None,
    "hop_state": None, "hop_channel": None, "hop_slots": None,
    "hop_missed": None, "hop_resyncs": None, "hop_streak": None,
    # AFC: the receiver tracks the ISS crystal instead of assuming a fixed
    # offset. offset_khz is the live tuning correction; cycles/moves say how
    # hard the loop is working.
    "afc_offset_khz": None, "afc_cycles": None, "afc_moves": None,
    "hop_status_epoch": None,
}
# Rain accumulation. The ISS only sends a 7-bit tip counter (wraps at 128) and
# a "time between tips" rate, so totals and history are OURS to keep - this is
# what the console does internally. Persisted to STATE_FILE.
rain = {
    "tips_total": 0,          # unwrapped, monotonic across restarts
    "last_counter": None,     # last raw 7-bit value seen
    "last_counter_epoch": None,
    "day": None,              # local date the today_tips total belongs to
    "today_tips": 0,
    "last_tip_epoch": None,
    "events": [],             # [[epoch, tips], ...] one entry per tip burst
}
rain_dirty = False
lock = threading.Lock()

# 10-minute rolling wind, the way the console presents it: every packet
# carries an instantaneous speed+direction (spiky), gust arrives in 0x9.
from collections import deque
import math
WIND_WINDOW = 600
wind_hist = deque()   # (epoch, speed_mph, dir_deg)
gust_hist = deque()   # (epoch, gust_mph)

def note_wind(now, speed, direction, gust=None):
    wind_hist.append((now, speed, direction))
    if gust is not None:
        gust_hist.append((now, gust))
    for dq in (wind_hist, gust_hist):
        while dq and dq[0][0] < now - WIND_WINDOW:
            dq.popleft()

def wind_rolling():
    """Mean speed, vector-mean direction, mean and peak gust over the window."""
    out = {}
    if wind_hist:
        n = len(wind_hist)
        out["wind_avg_10min_mph"] = round(sum(w[1] for w in wind_hist) / n, 1)
        # direction as a vector mean so 350 and 10 average to 0, not 180
        sx = sum(math.sin(math.radians(w[2])) for w in wind_hist)
        cy = sum(math.cos(math.radians(w[2])) for w in wind_hist)
        out["wind_dir_avg_10min_deg"] = round(math.degrees(math.atan2(sx, cy)) % 360)
    if gust_hist:
        out["gust_avg_10min_mph"] = round(sum(g[1] for g in gust_hist) / len(gust_hist), 1)
        out["gust_peak_10min_mph"] = max(g[1] for g in gust_hist)
    return out

def _today():
    return time.strftime("%Y-%m-%d")

def load_rain():
    try:
        with open(STATE_FILE) as f:
            saved = json.load(f)
    except FileNotFoundError:
        return
    except Exception as e:
        print(f"wxrx: could not read {STATE_FILE}: {e}", file=sys.stderr, flush=True)
        return
    for k in rain:
        if k in saved:
            rain[k] = saved[k]
    # A short restart should not lose the tips that happened while we were
    # down: keep the old counter as the delta baseline if it is recent. After
    # a long outage the 7-bit counter could have wrapped, so start fresh.
    if rain["last_counter_epoch"] and time.time() - rain["last_counter_epoch"] > 2 * 3600:
        rain["last_counter"] = None
    print(f"wxrx: rain state loaded, {rain['tips_total']} tips total, "
          f"{len(rain['events'])} logged bursts", flush=True)

def save_rain():
    global rain_dirty
    tmp = STATE_FILE + ".tmp"
    try:
        os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
        with open(tmp, "w") as f:
            json.dump(rain, f)
        os.replace(tmp, STATE_FILE)
        rain_dirty = False
    except Exception as e:
        print(f"wxrx: could not write {STATE_FILE}: {e}", file=sys.stderr, flush=True)

def note_rain_counter(c, now):
    """Fold one 7-bit counter reading into the running totals. Call under lock."""
    global rain_dirty
    today = _today()
    if rain["day"] != today:               # local-midnight rollover, like the console
        rain["day"], rain["today_tips"] = today, 0
        rain_dirty = True
    if rain["last_counter"] is not None:
        delta = (c - rain["last_counter"]) % 128
        # 64+ tips between two readings ~13 s apart is not weather, it is a
        # counter reset (ISS reboot) - re-baseline rather than log 0.64" of rain.
        if 0 < delta < 64:
            rain["tips_total"] += delta
            rain["today_tips"] += delta
            rain["last_tip_epoch"] = now
            rain["events"].append([round(now, 1), delta])
            cutoff = now - RAIN_LOG_DAYS * 86400
            rain["events"] = [e for e in rain["events"] if e[0] >= cutoff]
            rain_dirty = True
    rain["last_counter"], rain["last_counter_epoch"] = c, now

def rain_sums(now):
    """Inches in the last hour / 24 h / today, from the tip log."""
    h1 = sum(n for t, n in rain["events"] if t >= now - 3600)
    h24 = sum(n for t, n in rain["events"] if t >= now - 86400)
    today = rain["today_tips"] if rain["day"] == _today() else 0
    return {"rain_last_hour_in": round(h1 * TIP_INCHES, 2),
            "rain_24h_in": round(h24 * TIP_INCHES, 2),
            "rain_today_in": round(today * TIP_INCHES, 2),
            "rain_tips_total": rain["tips_total"],
            "rain_last_tip_epoch": rain["last_tip_epoch"]}

def rain_saver():
    # Tips are rare, so we also save on a timer to carry last_counter across
    # a restart; the per-tip save is what protects the totals.
    while True:
        time.sleep(300)
        with lock:
            if rain_dirty or rain["last_counter"] is not None:
                save_rain()

def decode(b, msg):
    """Return dict of fields from one 10-byte (bit-reversed) packet.

    Wind is present in EVERY packet; the rest rotate by message type.
    Confidence noted per field - temp was validated against the console."""
    out = {}
    out["wind_speed_mph"] = b[1]                      # every packet
    out["wind_direction_deg"] = round(b[2] * 360 / 255)  # every packet
    if msg == 0x8:      # temperature - VALIDATED (82.8 F matched console)
        out["temperature_f"] = (((b[3] << 8) | b[4]) >> 4) / 10.0
    elif msg == 0xA:    # humidity
        out["humidity_pct"] = (((b[4] >> 4) << 8) | b[3]) / 10.0
    elif msg == 0x9:    # wind gust
        out["wind_gust_mph"] = b[3]
    elif msg == 0xE:    # rain counter (monotonic tip count, wraps at 128)
        out["rain_counter"] = b[3] & 0x7F
    elif msg == 0x5:    # rain rate, as "time between bucket tips" - per the
        # weewx-meteostick driver, which matches the console. Verified against
        # our own no-rain frame (b3=FF b4=71 -> raw 0x3FF).
        raw = ((b[4] & 0x30) << 4) | b[3]
        if raw == 0x3FF:
            out["rain_tbt_s"], out["rain_rate_inph"] = None, 0.0
        else:
            tbt = raw / 16.0 if (b[4] & 0x40) == 0 else float(raw)   # heavy : light
            out["rain_tbt_s"] = round(tbt, 2)
            out["rain_rate_inph"] = round(3600.0 / tbt * TIP_INCHES, 3)
    elif msg in (0x7, 0x2):
        # 0x7 solar cell / panel output, 0x2 supercap voltage. Same 10-bit
        # layout, decoded the way weewx-meteostick does it (raw/300 = volts,
        # 0x3FF = not reported). Meteostick labels both "Vue only", yet this
        # VP2 ISS sends 0x7 with live data (~2.6 V in afternoon sun) and 0x2
        # as 0x3FF. The raw value is exported too, so if the /300 scaling
        # turns out wrong for a VP2 the history is still usable.
        raw = ((b[3] << 2) | (b[4] >> 6)) & 0x3FF
        volts = None if raw == 0x3FF else round(raw / 300.0, 3)
        if msg == 0x7:
            out["solar_raw"] = None if raw == 0x3FF else raw
            out["solar_voltage_v"] = volts
        else:
            out["supercap_voltage_v"] = volts
    # 0x4 UV and 0x6 solar radiation: sensors not fitted (b3 reads FF).
    return out

def reader():
    import serial
    while True:
        try:
            s = serial.Serial(DEVICE, BAUD, timeout=2)
            print(f"wxrx: opened {DEVICE}", flush=True)
            # NOTE: do NOT use `for raw in s:` - pyserial's iterator raises
            # StopIteration on a read timeout, so the loop exits every 2 s of
            # silence and the port gets reopened, missing packets in between.
            while True:
                raw = s.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()

                if line.startswith("STATUS"):
                    kv = dict(STATUS_KV.findall(line))
                    with lock:
                        if "state" in kv:
                            state["hop_state"] = kv["state"]
                        for key, field in (("ch", "hop_channel"), ("slots", "hop_slots"),
                                           ("missed", "hop_missed"), ("resyncs", "hop_resyncs"),
                                           ("streak", "hop_streak"),
                                           ("afc_cycles", "afc_cycles"),
                                           ("afc_moves", "afc_moves")):
                            if key in kv:
                                try:
                                    state[field] = int(kv[key])
                                except ValueError:
                                    pass
                        # The firmware reports Hz; everything else in this
                        # project talks kHz, so convert once, here.
                        if "offset_hz" in kv:
                            try:
                                state["afc_offset_khz"] = int(kv["offset_hz"]) / 1000.0
                            except ValueError:
                                pass
                        state["hop_status_epoch"] = time.time()
                    continue

                # davis-sweep results. SWEEP = one probe offset's yield;
                # SWEEPDONE = the peak of a completed pass.
                #
                # Every SWEEP point is logged WITH the outdoor and rack
                # temperatures of the moment, because the offset alone is
                # useless - the whole question is how the peak moves WITH
                # temperature, and joining that up afterwards from separate
                # series is exactly the kind of thing that goes wrong quietly.
                if line.startswith("SWEEP"):
                    kv = dict(STATUS_KV.findall(line))
                    done = line.startswith("SWEEPDONE")
                    rec = {"t": round(time.time(), 1), "done": done}
                    for k, v in kv.items():
                        try:
                            rec[k] = float(v) if "." in v else int(v)
                        except ValueError:
                            rec[k] = v
                    with lock:
                        rec["outdoor_f"] = state["temperature_f"]
                        rec["rack_f"] = (None if state["env_temp_c"] is None
                                         else round(state["env_temp_c"] * 9.0 / 5.0 + 32.0, 2))
                        if done:
                            state["sweep_pass"] = rec.get("pass")
                            state["sweep_best_khz"] = rec.get("best_khz")
                            state["sweep_best_abs_khz"] = rec.get("best_abs_khz")
                            state["sweep_best_good"] = rec.get("best_good")
                            state["sweep_pegged"] = 1 if "PEG" in kv else 0
                            state["sweep_epoch"] = time.time()
                    try:
                        with open(SWEEP_LOG, "a") as f:
                            f.write(json.dumps(rec) + "\n")
                    except Exception as e:
                        print(f"wxrx: sweep log write failed: {e}", file=sys.stderr, flush=True)
                    continue

                # RAK1906 line, e.g. "ENV t_c=23.41 rh=44.02 p_hpa=982.11".
                # Parsed with the same generic key=value scanner as STATUS, so
                # adding a field to the sketch needs no change here.
                if line.startswith("ENV"):
                    kv = dict(STATUS_KV.findall(line))
                    with lock:
                        for key, field in (("t_c", "env_temp_c"),
                                           ("rh", "env_humidity_pct"),
                                           ("p_hpa", "env_pressure_hpa")):
                            if key in kv:
                                try:
                                    state[field] = float(kv[key])
                                except ValueError:
                                    pass
                        if "t_c" in kv or "p_hpa" in kv:
                            state["env_epoch"] = time.time()
                    continue

                m = LINE.search(line)
                if not m:
                    continue
                ok = m.group(1) == "OK"
                with lock:
                    if not ok:
                        state["packets_bad"] += 1
                        continue
                    b = [int(x, 16) for x in m.group(2).split()]
                    msg = int(m.group(3), 16)
                    fields = decode(b, msg)
                    state.update(fields)
                    note_wind(time.time(), fields["wind_speed_mph"], fields["wind_direction_deg"],
                              fields.get("wind_gust_mph"))
                    if "rain_counter" in fields:
                        note_rain_counter(fields["rain_counter"], time.time())
                        if rain_dirty:
                            save_rain()
                    state["station_id"] = int(m.group(4))
                    # byte 0 low nibble: bits 0-2 = transmitter id, BIT 3 = battery low.
                    # Per the DavisRFM69 message-protocol table. This is the ISS's own
                    # hardware report, far better than inferring power health from gaps.
                    state["battery_low"] = 1 if (b[0] & 0x08) else 0
                    state["rssi_dbm"] = int(m.group(5))
                    state["last_msg_type"] = msg
                    state["last_packet_epoch"] = time.time()
                    state["packets_ok"] += 1
                    state["last_raw"] = m.group(2).strip()
                    # inventory: every type seen, with its most recent raw frame.
                    # This is what makes the "what have we got" view possible -
                    # undecoded types still show their bytes so scaling can be
                    # worked out later from real data rather than guessed.
                    mt = state["msgtypes"].setdefault(msg, {"count": 0})
                    mt["count"] += 1
                    mt["last_raw"] = m.group(2).strip()
                    mt["last_seen"] = time.time()
                    mt["b3"], mt["b4"], mt["b5"] = b[3], b[4], b[5]
        except Exception as e:
            print(f"wxrx: serial error: {e}; retrying in 5s", file=sys.stderr, flush=True)
            time.sleep(5)

ALTITUDE_M = float(os.environ.get("WXRX_ALTITUDE_M", "267"))

def sea_level_hpa(p_hpa, temp_c):
    """Station pressure -> sea-level pressure, the reduction weather services use
    and the one APRS `b` expects:

        P0 = P * (1 - (0.0065 h) / (T + 0.0065 h + 273.15)) ** -5.257

    T is the OUTDOOR temperature at the station, so this is handed the DAVIS
    reading, not the BME680's. The RAK sits in a sealed case in the basement and
    reads several degrees warm; using that here would bias every barometer
    reading we ever publish. The sensor's own temperature is a fallback only.
    At 267 m the correction is roughly +32 hPa, so getting it wrong is not subtle.
    """
    if p_hpa is None or temp_c is None:
        return None
    x = 0.0065 * ALTITUDE_M
    return round(p_hpa * (1 - x / (temp_c + x + 273.15)) ** -5.257, 2)

def f_to_c(f):
    return None if f is None else (f - 32.0) * 5.0 / 9.0

def dew_point_f(temp_f, rh_pct):
    """Dew point from temperature + relative humidity (Magnus, Alduchov-Eskridge
    coefficients: within ~0.1 C over -40..50 C). Derived, not measured - the ISS
    sends temp (msg 0x8) and humidity (msg 0xA); this is what a console shows."""
    if temp_f is None or rh_pct is None or rh_pct <= 0:
        return None
    t = (temp_f - 32.0) * 5.0 / 9.0
    a, b = 17.625, 243.04
    gamma = math.log(min(rh_pct, 100.0) / 100.0) + a * t / (b + t)
    td = b * gamma / (a - gamma)
    return round(td * 9.0 / 5.0 + 32.0, 1)


def metrics():
    with lock:
        st = dict(state)
    L = []
    def g(name, help_, val, typ="gauge"):
        if val is None:
            return
        L.append(f"# HELP {name} {help_}")
        L.append(f"# TYPE {name} {typ}")
        L.append(f"{name} {val}")
    g("davis_wind_speed_mph", "Wind speed", st["wind_speed_mph"])
    g("davis_wind_direction_degrees", "Wind direction", st["wind_direction_deg"])
    g("davis_temperature_fahrenheit", "Outside temperature", st["temperature_f"])
    g("davis_humidity_percent", "Outside relative humidity", st["humidity_pct"])
    g("davis_dew_point_fahrenheit", "Outside dew point, derived from temperature and humidity",
      dew_point_f(st["temperature_f"], st["humidity_pct"]))
    g("davis_wind_gust_mph", "Wind gust", st["wind_gust_mph"])
    # RAK1906. Named wxrx_env_* and NOT davis_*: these are RACK-ambient readings
    # from on top of the rack, and must never be mistaken for the ISS's outdoor
    # sensors. The rack's thermostatic fan is what drives their short-term shape.
    g("wxrx_env_temperature_fahrenheit", "Rack ambient temperature, top of rack (RAK1906); shaped by the rack's thermostatic fan",
      None if st["env_temp_c"] is None else round(st["env_temp_c"] * 9.0 / 5.0 + 32.0, 2))
    g("wxrx_env_humidity_percent", "Rack ambient relative humidity, top of rack (RAK1906)",
      st["env_humidity_pct"])
    g("wxrx_env_pressure_hpa", "Station barometric pressure as measured (RAK1906)",
      st["env_pressure_hpa"])
    g("wxrx_env_pressure_sealevel_hpa",
      f"Barometric pressure reduced to sea level from {ALTITUDE_M:.0f} m, using the Davis outdoor temperature",
      sea_level_hpa(st["env_pressure_hpa"], f_to_c(st["temperature_f"]) if st["temperature_f"] is not None
                                            else st["env_temp_c"]))
    # davis-sweep. best_abs_khz against davis_temperature_fahrenheit IS the
    # drift curve we are after; everything else here is context for it.
    g("wxrx_sweep_pass", "Completed davis-sweep passes since boot", st["sweep_pass"])
    g("wxrx_sweep_best_khz", "Probe offset with the best yield, relative to the -33 kHz baseline",
      st["sweep_best_khz"])
    g("wxrx_sweep_best_abs_khz", "Total offset from the nominal hop table at the sweep peak",
      st["sweep_best_abs_khz"])
    g("wxrx_sweep_best_good", "Good packets at the peak offset, out of the probe slots tried",
      st["sweep_best_good"])
    g("wxrx_sweep_pegged", "1 = peak sat at an end of the sweep, so the real centre is further out",
      st["sweep_pegged"])
    g("wxrx_sweep_age_seconds", "Seconds since the last completed sweep pass",
      None if st["sweep_epoch"] is None else round(time.time() - st["sweep_epoch"], 1))
    g("wxrx_env_age_seconds", "Seconds since the last RAK1906 reading",
      None if st["env_epoch"] is None else round(time.time() - st["env_epoch"], 1))
    with lock:
        wr = wind_rolling()
    g("davis_wind_avg_10min_mph", "10-minute mean wind speed", wr.get("wind_avg_10min_mph"))
    g("davis_wind_dir_avg_10min_degrees", "10-minute vector-mean wind direction",
      wr.get("wind_dir_avg_10min_deg"))
    g("davis_gust_avg_10min_mph", "10-minute mean of reported gusts", wr.get("gust_avg_10min_mph"))
    g("davis_gust_peak_10min_mph", "10-minute peak gust", wr.get("gust_peak_10min_mph"))
    g("davis_rain_counter", "Rain tip counter (wraps at 128)", st["rain_counter"])
    g("davis_rain_rate_inph", "Rain rate reported by the ISS, inches per hour",
      st["rain_rate_inph"])
    with lock:
        rs = rain_sums(time.time())
    if rain["last_counter"] is not None:
        g("davis_rain_tips_total", "Bucket tips, unwrapped and persisted across restarts",
          rs["rain_tips_total"], "counter")
        g("davis_rain_today_inches", "Rain since local midnight", rs["rain_today_in"])
        g("davis_rain_last_hour_inches", "Rain in the last 60 minutes", rs["rain_last_hour_in"])
        g("davis_rain_24h_inches", "Rain in the last 24 hours", rs["rain_24h_in"])
        g("davis_rain_last_tip_timestamp_seconds", "Unix time of the last bucket tip",
          rs["rain_last_tip_epoch"])
    g("davis_rssi_dbm", "RSSI of last packet", st["rssi_dbm"])
    g("davis_last_packet_timestamp_seconds", "Unix time of last valid packet",
      st["last_packet_epoch"])
    g("davis_packets_total_ok", "Packets passing CRC", st["packets_ok"], "counter")
    g("davis_packets_total_bad", "Packets failing CRC", st["packets_bad"], "counter")
    # age is what actually matters on a dashboard: a stale feed must look stale
    if st["last_packet_epoch"]:
        g("davis_packet_age_seconds", "Seconds since last valid packet",
          round(time.time() - st["last_packet_epoch"], 1))
    g("davis_battery_low", "ISS battery-low flag reported in byte 0 bit 3 (1 = low)",
      st["battery_low"])
    g("davis_up", "1 if a valid packet has been seen", 1 if st["packets_ok"] else 0)
    g("davis_solar_voltage_volts",
      "ISS solar panel output from message 0x7, raw/300 per weewx-meteostick (scaling unverified on VP2)",
      st["solar_voltage_v"])
    g("davis_solar_raw", "ISS message 0x7 10-bit raw value, 0x3FF (absent) suppressed", st["solar_raw"])
    g("davis_supercap_voltage_volts",
      "ISS supercap voltage from message 0x2, raw/300 per weewx-meteostick; absent when the ISS reports 0x3FF",
      st["supercap_voltage_v"])

    for mt, d in sorted(st["msgtypes"].items()):
        L.append("# HELP davis_msgtype_total Packets seen per ISS message type")
        L.append("# TYPE davis_msgtype_total counter")
        L.append(f'davis_msgtype_total{{type="0x{mt:X}"}} {d["count"]}')

    # ---- hop follower (milestone 2). Absent entirely on milestone-1 firmware,
    # which is deliberate: a missing metric is honest about which sketch is
    # running, whereas a zero would read as "tracking, badly".
    if st["hop_state"] is not None:
        g("davis_hop_tracking", "1 if following the hop sequence, 0 if reacquiring",
          1 if st["hop_state"] == "TRACK" else 0)
        g("davis_hop_channel", "Current hop sequence index (0-50)", st["hop_channel"])
        g("davis_hop_slots_total", "Transmit slots accounted for", st["hop_slots"], "counter")
        g("davis_hop_missed_total", "Slots where no valid packet arrived",
          st["hop_missed"], "counter")
        g("davis_hop_resyncs_total", "Times lock was lost and reacquired",
          st["hop_resyncs"], "counter")
        g("davis_hop_miss_streak", "Consecutive missed slots right now", st["hop_streak"])
    # The receiver's live tuning correction. This is a MEASUREMENT of where the
    # ISS transmitter actually is, not a setting: the AFC loop walks it to
    # wherever the packets are, so plotting it against outdoor temperature gives
    # the crystal's tempco curve continuously, without running a sweep.
    g("davis_afc_offset_khz",
      "Live AFC tuning offset from the nominal hop table, kHz", st["afc_offset_khz"])
    g("davis_afc_cycles_total", "AFC tracking cycles run since boot",
      st["afc_cycles"], "counter")
    g("davis_afc_moves_total", "AFC cycles that actually moved the offset",
      st["afc_moves"], "counter")
    return "\n".join(L) + "\n"

PAGE = """<!doctype html><meta charset=utf-8><title>wxrx &mdash; Davis Vantage Pro2</title>
<meta name=viewport content="width=device-width,initial-scale=1">
<style>
:root{--bg:#111;--panel:#1a1a1a;--line:#2c2c2c;--fg:#e8e8e8;--dim:#8a8a8a;--ok:#8ddc8d;--warn:#e0c060;--bad:#e08080;--blue:#3987e5}
*{box-sizing:border-box}
body{font:15px/1.45 system-ui,sans-serif;background:var(--bg);color:var(--fg);margin:0;padding:1.5rem}
h1{font-size:1.15rem;margin:0 0 .25rem;color:var(--ok);font-weight:600}
h2{font-size:.8rem;letter-spacing:.08em;text-transform:uppercase;color:var(--dim);
   margin:1.75rem 0 .6rem;font-weight:600}
.sub{color:var(--dim);font-size:.85rem;margin-bottom:.5rem}
.grid{display:grid;gap:.75rem;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));align-content:start}
.card{background:var(--panel);border:1px solid var(--line);border-radius:7px;padding:.7rem .85rem}
.k{color:var(--dim);font-size:.75rem;letter-spacing:.04em;text-transform:uppercase}
.v{font-size:1.5rem;font-weight:600;color:var(--ok);line-height:1.25;margin-top:.15rem}
.v small{font-size:.8rem;color:var(--dim);font-weight:400}
.v.na{color:var(--dim);font-size:1rem;font-weight:400}
.v.rain{color:var(--blue)}
.s{color:var(--dim);font-size:.78rem;margin-top:.1rem}
/* current conditions: cards left, compass right */
.now{display:grid;grid-template-columns:1fr 480px;gap:.75rem;align-items:stretch}
/* left column: the two card grids share the wind block's height */
.now>div:first-child{display:flex;flex-direction:column;gap:.75rem}
.now>div:first-child .grid{align-content:stretch}
/* temperature + humidity are the hero figures; everything else shrinks to fit */
.hero{grid-template-columns:1fr 1fr !important;flex:3}
.hero .k{font-size:.85rem}
.hero .v{font-size:3.6rem;line-height:1.1}
.hero .v small{font-size:1.1rem}
.hero .v.na{font-size:1.2rem;font-weight:400}
.small{grid-template-columns:repeat(auto-fit,minmax(78px,1fr)) !important;flex:1}
.small .card{padding:.45rem .5rem}
.small .k{font-size:.66rem}
.small .v{font-size:1.15rem}
.small .v small{font-size:.7rem}
.small .v.na{font-size:.85rem}
.now .grid .card{display:flex;flex-direction:column;justify-content:center}
.rosewrap{display:flex;gap:.75rem;align-items:center}
.rosewrap svg{width:236px;flex:none}
.windnums{display:flex;flex-direction:column;gap:.5rem;flex:1;min-width:0}
.windnums .card{padding:.45rem .7rem;background:#161616}
@media (max-width:1100px){.now{grid-template-columns:1fr}}
.now .grid{grid-template-columns:repeat(auto-fit,minmax(135px,1fr));margin-top:0 !important}
.rose{display:flex;flex-direction:column}
.rose svg{flex:1;min-height:0}
@media (max-width:560px){.rosewrap{flex-direction:column}}
/* charts */
.chart{background:var(--panel);border:1px solid var(--line);border-radius:7px;padding:.7rem .85rem}
.bar{display:flex;justify-content:space-between;flex-wrap:wrap;gap:.5rem;margin-bottom:.7rem}
.tabs{display:flex;flex-wrap:wrap;gap:.3rem}
.tabs button{background:#222;border:1px solid var(--line);color:var(--dim);border-radius:6px;
  padding:.45rem .95rem;font:inherit;font-size:.9rem;font-weight:600;cursor:pointer}
.tabs button:hover{color:var(--fg);border-color:#444}
.tabs button.on{color:#fff;background:var(--blue);border-color:var(--blue)}
.ranges button{padding:.45rem .7rem;font-weight:500}
.ranges button.on{background:#333;border-color:#555;color:var(--fg)}
.chart .k{margin-bottom:.35rem;display:flex;justify-content:space-between;align-items:baseline}
.legend{font-size:.72rem;color:var(--dim);text-transform:none;letter-spacing:0}
svg{display:block;width:100%;height:auto;font:11px system-ui,sans-serif}
.ax{fill:var(--dim)} .gl{stroke:var(--line);stroke-width:1}
.tip{position:fixed;pointer-events:none;background:#222;border:1px solid #444;border-radius:5px;padding:.35rem .55rem;font-size:.78rem;color:var(--fg);display:none;z-index:9}
table{border-collapse:collapse;width:100%;font-size:.85rem}
th{text-align:left;color:var(--dim);font-weight:600;font-size:.72rem;letter-spacing:.05em;
   text-transform:uppercase;padding:.4rem .6rem;border-bottom:1px solid var(--line)}
td{padding:.4rem .6rem;border-bottom:1px solid #202020;vertical-align:top}
tr.never td{color:#5a5a5a}
code{font-family:ui-monospace,Menlo,monospace;font-size:.82rem;color:#b9c9d9}
.tag{display:inline-block;font-size:.68rem;padding:.1rem .4rem;border-radius:3px;
     letter-spacing:.04em;text-transform:uppercase;font-weight:600}
.t-dec{background:#1e3a1e;color:var(--ok)}
.t-raw{background:#3a331e;color:var(--warn)}
.t-unk{background:#2a2a2a;color:#777}
.t-nofit{background:#33232a;color:#c98fa4}
.foot{color:var(--dim);font-size:.8rem;margin-top:1.5rem;border-top:1px solid var(--line);padding-top:.7rem}
</style>
<h1>Davis Vantage Pro2 &mdash; ISS receiver</h1>
<div class=sub id=hdr>&nbsp;</div>

<h2>Current conditions</h2>
<div class=now>
 <div>
  <div class="grid hero" id=wx></div>
  <div class="grid small" id=rain></div>
 </div>
 <div class="card rose"><div class=k>Wind</div>
  <div class=rosewrap>
   <svg id=rosesvg viewBox="0 0 250 270" aria-label="Wind rose, last 24 hours"></svg>
   <div class=windnums id=windnums></div>
  </div>
  <div class=s id=rosenote style="text-align:center"></div></div>
</div>

<h2>Weather history</h2>
<div class=chart id=wxchart>
 <div class=bar><div class=tabs></div><div class="tabs ranges"></div></div>
 <div class=k><span class=title>&nbsp;</span><span class="legend note"></span></div>
 <svg viewBox="0 0 800 300" aria-label="Weather history"></svg>
</div>

<h2>Link &amp; receiver health</h2>
<div class=grid id=rf></div>
<div class=chart id=rfchart style="margin-top:.75rem">
 <div class=bar><div class=tabs></div><div class="tabs ranges"></div></div>
 <div class=k><span class=title>&nbsp;</span><span class="legend note"></span></div>
 <svg viewBox="0 0 800 300" aria-label="Link history"></svg>
</div>
<div class=tip id=tip></div>

<h2>Message inventory &mdash; what the ISS actually transmits</h2>
<div class=sub>Wind speed and direction ride in <em>every</em> packet; the rest rotate by type.
Undecoded types show their raw bytes so scaling can be derived from real data.
&ldquo;Sensor not fitted&rdquo; means the ISS sends the message but the optional sensor
is not installed &mdash; a hardware gap, not a decoding one.</div>
<table id=mt><thead><tr><th>Type</th><th>Meaning</th><th>Status</th><th>Count</th>
<th>Last seen</th><th>Raw frame</th></tr></thead><tbody></tbody></table>

<div class=foot id=foot></div>

<script>
// Message types per the DavisRFM69 project + our own IQ analysis. "decoded" means
// wxrx parses it; "raw" means we know what it is but have not confirmed scaling.
// 'nofit' = the ISS transmits this type but the optional sensor is not installed
// (confirmed by the owner; byte 3 reads FF). Those sensors plug into the ISS
// board and cost ~$400, so this is a hardware gap, not a decoding gap.
const TYPES={0x2:['Supercap voltage','dec'],0x4:['UV index','nofit'],0x5:['Rain rate','dec'],
 0x6:['Solar radiation','nofit'],0x7:['Solar panel voltage','dec'],0x8:['Temperature','dec'],
 0x9:['Wind gust','dec'],0xA:['Humidity','dec'],0xE:['Rain counter','dec']};
let latest=null;
const PTS16=['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSW','SW','WSW','W','WNW','NW','NNW'];
const compass=deg=>PTS16[Math.round(deg/22.5)%16];
const DAYS=['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
const ago=s=>s==null?'&mdash;':s<60?s.toFixed(0)+'s':s<3600?(s/60).toFixed(1)+'m':(s/3600).toFixed(1)+'h';
// Same as ago(), but rolls over into days. "Last tip" is really "time since it
// last rained", which in a dry spell runs to days - and "63.4h" is something you
// have to do arithmetic on before it means anything.
const agoLong=s=>{ if(s==null) return '&mdash;'; if(s<86400) return ago(s);
  const d=Math.floor(s/86400), h=Math.floor((s-d*86400)/3600); return `${d}d ${h}h`; };
const fmtIn=v=>v==null?null:v.toFixed(2);
const hhmm=t=>{ const d=new Date(t*1000); return d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0'); };
const card=(k,v,u,na,sub)=>`<div class=card><div class=k>${k}</div>`+
  (v==null?`<div class="v na">${na||'no data yet'}</div>`
         :`<div class=v>${v}<small> ${u||''}</small></div>`)+(sub?`<div class=s>${sub}</div>`:'')+`</div>`;
const showTip=(e,html)=>{ tip.innerHTML=html; tip.style.display='block'; tip.style.left=(e.clientX+14)+'px'; tip.style.top=(e.clientY+14)+'px'; };
const hideTip=()=>tip.style.display='none';

// ---------------- current values, every 3 s ----------------
let tick=async function(){
 let d; try{ d=await (await fetch('/api/current.json')).json(); }catch(e){ return; }
 latest=d;
 const age=d.packet_age_seconds;
 hdr.innerHTML=`station ${d.station_id ?? '?'} &middot; last packet ${ago(age)} ago &middot; `+
   `${d.packets_ok} good / ${d.packets_bad} bad`;

 const slp=d.pressure_sealevel_hpa;
 wx.innerHTML=card('Temperature',d.temperature_f,'&deg;F')
  +card('Humidity',d.humidity_pct,'%',null,d.dew_point_f==null?'':`dew point ${d.dew_point_f}&deg;F`)
  // Sea-level pressure is the meteorologically meaningful number and the one the
  // APRS beacon sends; station pressure and inHg ride along underneath.
  +card('Pressure',slp==null?null:slp.toFixed(1),'hPa','no sensor',
        slp==null?'':`${(slp*0.02953).toFixed(2)} inHg &middot; station ${d.env_pressure_hpa==null?'&mdash;':d.env_pressure_hpa.toFixed(1)} hPa`);
 windnums.innerHTML=
   card('Wind',d.wind_avg_10min_mph,'mph',null,`10-min mean &middot; now ${d.wind_speed_mph ?? '&mdash;'} mph`)
  +card('Direction',d.wind_dir_avg_10min_deg==null?null:`${compass(d.wind_dir_avg_10min_deg)} ${d.wind_dir_avg_10min_deg}&deg;`,'',null,`10-min mean &middot; now ${d.wind_direction_deg==null?'&mdash;':compass(d.wind_direction_deg)+' '+d.wind_direction_deg+'&deg;'}`)
  +card('Gust',d.gust_avg_10min_mph,'mph',null,`10-min mean &middot; peak ${d.gust_peak_10min_mph ?? '&mdash;'} mph`);

 const rate=d.rain_rate_inph, lastTip=d.rain_last_tip_epoch;
 const recentTip=lastTip!=null && (Date.now()/1000-lastTip)<600;
 const raining=(rate!=null&&rate>0)||recentTip;
 rain.innerHTML=
   `<div class=card><div class=k>Rain</div><div class="v${raining?' rain':''}">${raining?'RAINING':'Dry'}</div></div>`
  +card('Rate',rate==null?null:rate.toFixed(2),'in/h')
  +card('Last hour',fmtIn(d.rain_last_hour_in),'in')
  +card('Today',fmtIn(d.rain_today_in),'in')
  +card('Last 24 h',fmtIn(d.rain_24h_in),'in')
  +card('Last tip',lastTip==null?null:agoLong(Date.now()/1000-lastTip),'ago','none logged')
  +card('Tips',d.rain_tips_total,'total');

 const yieldPct=(d.hop_slots&&d.hop_slots>0)
   ? (100*(d.hop_slots-(d.hop_missed||0))/d.hop_slots).toFixed(2) : null;
 rf.innerHTML=card('RSSI',d.rssi_dbm,'dBm')+card('Packet age',age==null?null:age.toFixed(1),'s')
  +card('Hop state',d.hop_state?d.hop_state:null,'',(d.hop_state||'&mdash;'))
  +card('Hop channel',d.hop_channel,'of 51')
  +card('Slot yield',yieldPct,'%')+card('Resyncs',d.hop_resyncs,'')
  +card('AFC offset',d.afc_offset_khz==null?null:d.afc_offset_khz.toFixed(1),'kHz','not tracking',
        d.afc_cycles==null?'':`${d.afc_moves ?? 0} moves in ${d.afc_cycles} cycles`)
  +card('ISS battery',d.battery_low==null?null:(d.battery_low?'LOW':'OK'),'')
  +card('Solar panel',d.solar_voltage_v==null?null:d.solar_voltage_v.toFixed(2),'V','not reported');

 const now=Date.now()/1000, rows=[];
 for(let t=0;t<16;t++){
   const info=TYPES[t], seen=(d.msgtypes||{})[t]||(d.msgtypes||{})[String(t)];
   if(!info && !seen) continue;                    // hide types we neither know nor have seen
   const [name,kind]=info||['unknown','unk'];
   const tag=kind==='nofit'?'<span class="tag t-nofit">sensor not fitted</span>'
        :!seen?'<span class="tag t-unk">never seen</span>'
        :kind==='dec'?'<span class="tag t-dec">decoded</span>'
                     :'<span class="tag t-raw">raw only</span>';
   rows.push(`<tr class="${seen?'':'never'}"><td><code>0x${t.toString(16).toUpperCase()}</code></td>`+
     `<td>${name}</td><td>${tag}</td><td>${seen?seen.count:'&mdash;'}</td>`+
     `<td>${seen?ago(now-seen.last_seen):'&mdash;'}</td>`+
     `<td><code>${seen?seen.last_raw:'&mdash;'}</code></td></tr>`);
 }
 mt.tBodies[0].innerHTML=rows.join('');
 foot.innerHTML=`bytes 0-5 are payload, 6-7 CRC-16/CCITT, 8-9 repeater (FF FF = direct). `+
   `Byte 1 = wind speed, byte 2 = wind direction, in every frame. `+
   `Refreshing every 3s.`;
}

// ---------------- history data ----------------
async function fetchSeries(tab, hours){
 if(tab.kind==='rainacc'){
   const r=await (await fetch(`/api/rain.json?hours=${hours}`)).json();
   const now=r.now, start=now-hours*3600, ev=r.events.filter(e=>e[0]>=start).sort((a,b)=>a[0]-b[0]);
   let s=0; const pts=[[start,0]]; for(const [t,n] of ev){ pts.push([t,s]); s+=n*r.tip_inches; pts.push([t,s]); }
   pts.push([now,s]); return {pts,start,end:now,note:`${s.toFixed(2)} in total · ${ev.reduce((a,e)=>a+e[1],0)} tips`};
 }
 const h=await (await fetch(`/api/history?metric=${tab.metric}&hours=${hours}${tab.avg?'&avg='+tab.avg:''}`)).json();
 if(h.error) return {error:h.error};
 const now=Date.now()/1000, v=h.values.map(p=>p[1]);
 const last=v[v.length-1];
 const note=v.length?`last ${last.toFixed(tab.dec)}${tab.ref?` (${(last/tab.ref.v*100).toFixed(1)} % of max)`:''} · min ${Math.min(...v).toFixed(tab.dec)} · max ${Math.max(...v).toFixed(tab.dec)} ${tab.unit}`:'';
 return {pts:h.values,start:now-hours*3600,end:now,note};
}
async function fetchRose(hours){
 const [dh,sh]=await Promise.all(['davis_wind_direction_degrees','davis_wind_speed_mph']
   .map(m=>fetch(`/api/history?metric=${m}&hours=${hours}`).then(r=>r.json())));
 if(dh.error||sh.error) return {error:dh.error||sh.error};
 const sp=new Map(sh.values.map(([t,v])=>[t,v]));
 const sect=Array.from({length:16},()=>({n:0,sum:0})); let calm=0,total=0;
 for(const [t,dir] of dh.values){ const v=sp.get(t); if(v==null) continue; total++;
   if(v<1){ calm++; continue; } const i=Math.round(dir/22.5)%16; sect[i].n++; sect[i].sum+=v; }
 return {sect,calm,total};
}

// Join two series into (x,y) pairs. The history proxy aligns every series to the
// same step (see history(): start is floored to it), which is what makes this a
// plain Map lookup rather than a nearest-time search - the wind rose already
// relies on the same guarantee.
async function fetchXY(tab, hours){
 const [xh,yh]=await Promise.all([tab.xmetric,tab.ymetric]
   .map(m=>fetch(`/api/history?metric=${m}&hours=${hours}`).then(r=>r.json())));
 if(xh.error||yh.error) return {error:xh.error||yh.error};
 const ym=new Map(yh.values.map(([t,v])=>[t,v]));
 const pts=[];
 for(const [t,xv] of xh.values){ const yv=ym.get(t); if(yv!=null) pts.push([xv,yv,t]); }
 return {pts};
}

// ---------------- line/area chart into an svg ----------------
const niceStep=(r,dec)=>{ const p=Math.pow(10,Math.floor(Math.log10(r/4||1))); for(const m of ((dec||p>=10)?[1,2,2.5,5,10]:[1,2,5,10])) if(m*p*4>=r) return m*p; return 10*p; };
function drawLine(svg, tab, d, hours){
 const W=800,H=300,L=48,R=12,T=22,BOT=24,PH=H-T-BOT, pw=W-L-R;
 if(d.error||!d.pts.length){ svg.innerHTML=`<text class=ax x=${W/2} y=${H/2} text-anchor=middle>${d.error||'no data yet'}</text>`; return; }
 const {pts,start,end}=d;
 let vmin=Math.min(...pts.map(p=>p[1])), vmax=Math.max(...pts.map(p=>p[1]));
 if(tab.from0) vmin=0; if(tab.max!=null) vmax=Math.max(vmax,tab.max);
 const st=tab.step||niceStep(Math.max(vmax-vmin,1e-6),tab.dec); vmin=Math.floor(vmin/st)*st; vmax=Math.ceil(vmax/st)*st;
 if(vmax===vmin) vmax=vmin+st; const ticks=[]; for(let v=vmin;v<=vmax+1e-9;v+=st) ticks.push(+v.toFixed(6));
 const x=t=>L+(t-start)/(end-start)*pw, y=v=>T+PH-(v-vmin)/(vmax-vmin)*PH;
 let g='';
 for(const v of ticks) g+=`<line class=gl x1="${L}" x2="${W-R}" y1="${y(v)}" y2="${y(v)}"/>`+
   `<text class=ax x=${L-6} y=${y(v)+4} text-anchor=end>${v.toFixed(tab.dec)}</text>`;
 g+=`<text class=ax x=${L} y=${T-5}>${tab.unit}</text>`;
 if(tab.ref){ const ry=y(tab.ref.v);
   g+=`<line x1="${L}" x2="${W-R}" y1="${ry}" y2="${ry}" stroke="#8a8a8a" stroke-width="1" stroke-dasharray="4 3"/>`+
      `<text class=ax x=${W-R} y=${ry-4} text-anchor=end>${tab.ref.label}</text>`; }
 // x ticks: 1 h / 3 h / 12 h / 24 h depending on range; midnight ticks show the weekday
 const tick=hours<=6?3600:hours<=24?10800:hours<=72?43200:86400, off=new Date().getTimezoneOffset()*60;
 for(let t=Math.ceil((start-off)/tick)*tick+off;t<=end;t+=tick){ const dd=new Date(t*1000);
   const lab=(tick>=43200&&dd.getHours()===0)?DAYS[dd.getDay()]:hhmm(t);
   g+=`<line class=gl x1="${x(t)}" x2="${x(t)}" y1="${T+PH}" y2="${T+PH+4}"/>`+
      `<text class=ax x=${x(t)} y=${H-7} text-anchor=middle>${lab}</text>`; }
 // mark specs: 2px line, area = 10% wash, hover dot with a surface ring
 const poly=pts.map(([t,v])=>`${x(t).toFixed(1)},${y(v).toFixed(1)}`).join(' ');
 if(tab.kind==='area'||tab.kind==='rainacc') g+=`<polygon fill="#3987e5" fill-opacity=".1" points="${L},${y(vmin)} ${poly} ${x(end)},${y(vmin)}"/>`;
 g+=`<polyline fill=none stroke="#3987e5" stroke-width=2 stroke-linejoin=round stroke-linecap=round points="${poly}"/>`;
 g+=`<line class="gl xh" x1=0 x2=0 y1="${T}" y2="${T+PH}" stroke="#666" style="display:none"/>`+
    `<circle class=xd r="4" fill="#3987e5" stroke="#1a1a1a" stroke-width="2" style="display:none"/>`+
    `<rect class=hit x="${L}" y="${T}" width="${pw}" height="${PH}" fill="transparent"/>`;
 svg.innerHTML=g;
 const hit=svg.querySelector('.hit'), xh=svg.querySelector('.xh'), xd=svg.querySelector('.xd');
 hit.onmousemove=e=>{ const rect=svg.getBoundingClientRect(), sx=W/rect.width;
   const t=start+((e.clientX-rect.left)*sx-L)/pw*(end-start);
   let lo=0,hi=pts.length-1; while(lo<hi){ const m=(lo+hi)>>1; if(pts[m][0]<t) lo=m+1; else hi=m; }
   const p=pts[lo]&&(!pts[lo-1]||Math.abs(pts[lo][0]-t)<Math.abs(pts[lo-1][0]-t))?pts[lo]:pts[lo-1];
   xh.setAttribute('x1',x(p[0])); xh.setAttribute('x2',x(p[0])); xh.style.display='';
   xd.setAttribute('cx',x(p[0])); xd.setAttribute('cy',y(p[1])); xd.style.display='';
   const pd=new Date(p[0]*1000);
   showTip(e,`<b>${tab.title||tab.label}</b><br>${hours>24?DAYS[pd.getDay()]+' ':''}${hhmm(p[0])}<br>${p[1].toFixed(tab.dec)} ${tab.unit}${tab.ref?` · ${(p[1]/tab.ref.v*100).toFixed(1)} % of max`:''}`); };
 hit.onmouseleave=()=>{ hideTip(); xh.style.display='none'; xd.style.display='none'; };
}

// ---------------- scatter: one metric against another ----------------
// Time is NOT an axis here - every sample in the window is one point, x = the
// driver, y = the response. That is the only way to SEE a threshold: on a time
// chart a knee is just another slope, and the eye cannot separate it from the
// day/night cycle it rides on.
// Blue dots are the raw samples; the pale line is the binned mean, drawn in the
// text colour so it reads as an annotation and not as a second data series
// (same rule as the rose needle).
function drawScatter(svg, tab, d){
 const W=800,H=300,L=48,R=12,T=22,BOT=34,PH=H-T-BOT,pw=W-L-R;
 if(d.error||!d.pts.length){ svg.innerHTML=`<text class=ax x=${W/2} y=${H/2} text-anchor=middle>${d.error||'no data yet'}</text>`; return ''; }
 const xs=d.pts.map(p=>p[0]), ys=d.pts.map(p=>p[1]);
 let xmin=Math.min(...xs), xmax=Math.max(...xs);
 const xst=niceStep(Math.max(xmax-xmin,1e-6),tab.xdec); xmin=Math.floor(xmin/xst)*xst; xmax=Math.ceil(xmax/xst)*xst;
 if(xmax===xmin) xmax=xmin+xst;
 let ymin=tab.from0?0:Math.min(...ys), ymax=Math.max(...ys); if(tab.max!=null) ymax=Math.max(ymax,tab.max);
 const yst=tab.step||niceStep(Math.max(ymax-ymin,1e-6),tab.dec); ymin=Math.floor(ymin/yst)*yst; ymax=Math.ceil(ymax/yst)*yst;
 if(ymax===ymin) ymax=ymin+yst;
 const x=v=>L+(v-xmin)/(xmax-xmin)*pw, y=v=>T+PH-(v-ymin)/(ymax-ymin)*PH;
 let g='';
 for(let v=ymin;v<=ymax+1e-9;v+=yst) g+=`<line class=gl x1="${L}" x2="${W-R}" y1="${y(v)}" y2="${y(v)}"/>`+
   `<text class=ax x=${L-6} y=${y(v)+4} text-anchor=end>${v.toFixed(tab.dec)}</text>`;
 g+=`<text class=ax x=${L} y=${T-5}>${tab.unit}</text>`;
 for(let v=xmin;v<=xmax+1e-9;v+=xst) g+=`<line class=gl x1="${x(v)}" x2="${x(v)}" y1="${T+PH}" y2="${T+PH+4}"/>`+
   `<text class=ax x=${x(v)} y=${H-17} text-anchor=middle>${v.toFixed(tab.xdec)}</text>`;
 g+=`<text class=ax x=${(L+W-R)/2} y=${H-3} text-anchor=middle>${tab.xlabel}</text>`;
 // ceiling reference (e.g. 23.4/min = every slot heard)
 if(tab.ref){ const ry=y(tab.ref.v);
   g+=`<line x1="${L}" x2="${W-R}" y1="${ry}" y2="${ry}" stroke="#8a8a8a" stroke-width="1" stroke-dasharray="4 3"/>`+
      `<text class=ax x=${W-R} y=${ry-4} text-anchor=end>${tab.ref.label}</text>`; }
 // binned mean + the convergence point: the coldest bin whose mean is already
 // within CONV of the ceiling. That temperature is the answer to "how cold is
 // too cold", which is the whole reason this tab exists.
 const NB=14, bw=(xmax-xmin)/NB, bins=Array.from({length:NB},()=>[]);
 for(const [xv,yv] of d.pts){ const i=Math.min(NB-1,Math.max(0,Math.floor((xv-xmin)/bw))); bins[i].push(yv); }
 const CONV=0.95, ceil=tab.ref?tab.ref.v:Math.max(...ys);
 const mean=[]; let conv=null;
 for(let i=0;i<NB;i++){ if(bins[i].length<3) continue;
   const m=bins[i].reduce((a,b)=>a+b,0)/bins[i].length, c=xmin+(i+0.5)*bw;
   mean.push([c,m,bins[i].length]);
   if(conv===null && m>=ceil*CONV) conv=c; else if(m<ceil*CONV) conv=null; }
 if(conv!=null){ g+=`<line x1="${x(conv)}" x2="${x(conv)}" y1="${T}" y2="${T+PH}" stroke="#8a8a8a" stroke-width="1" stroke-dasharray="4 3"/>`+
   `<text class=ax x=${x(conv)+4} y=${T+11}>converges ~${conv.toFixed(tab.xdec)} ${tab.xunit}</text>`; }
 for(const [xv,yv] of d.pts) g+=`<circle cx="${x(xv).toFixed(1)}" cy="${y(yv).toFixed(1)}" r="3" fill="#3987e5" fill-opacity=".45"/>`;
 if(mean.length>1) g+=`<polyline fill=none stroke="#e8e8e8" stroke-width=2 stroke-linejoin=round stroke-linecap=round points="${mean.map(([c,m])=>`${x(c).toFixed(1)},${y(m).toFixed(1)}`).join(' ')}"/>`;
 g+=`<circle class=xd r="5" fill="none" stroke="#e8e8e8" stroke-width="2" style="display:none"/>`+
    `<rect class=hit x="${L}" y="${T}" width="${pw}" height="${PH}" fill="transparent"/>`;
 svg.innerHTML=g;
 const hit=svg.querySelector('.hit'), xd=svg.querySelector('.xd');
 hit.onmousemove=e=>{ const rect=svg.getBoundingClientRect(), sc=W/rect.width;
   const px=(e.clientX-rect.left)*sc, py=(e.clientY-rect.top)*sc;
   let best=null,bd=1e9;
   for(const p of d.pts){ const dx=x(p[0])-px, dy=y(p[1])-py, dd=dx*dx+dy*dy; if(dd<bd){ bd=dd; best=p; } }
   if(!best||bd>900){ hideTip(); xd.style.display='none'; return; }
   xd.setAttribute('cx',x(best[0])); xd.setAttribute('cy',y(best[1])); xd.style.display='';
   const pd=new Date(best[2]*1000);
   showTip(e,`<b>${tab.title||tab.label}</b><br>${DAYS[pd.getDay()]} ${hhmm(best[2])}<br>`+
             `${best[0].toFixed(tab.xdec)} ${tab.xunit} &rarr; ${best[1].toFixed(tab.dec)} ${tab.unit}`); };
 hit.onmouseleave=()=>{ hideTip(); xd.style.display='none'; };
 // Pearson r, so the strength of the relationship is a number on the page and
 // not an impression from the dots.
 const n=xs.length, mx=xs.reduce((a,b)=>a+b,0)/n, my=ys.reduce((a,b)=>a+b,0)/n;
 let sxy=0,sxx=0,syy=0;
 for(let i=0;i<n;i++){ const a=xs[i]-mx, b=ys[i]-my; sxy+=a*b; sxx+=a*a; syy+=b*b; }
 const r=(sxx&&syy)?sxy/Math.sqrt(sxx*syy):0;
 return `${n} samples · r = ${r>=0?'+':''}${r.toFixed(2)}`+
        (conv!=null?` · converges to ${ceil.toFixed(tab.dec)} ${tab.unit} at ~${conv.toFixed(tab.xdec)} ${tab.xunit}`:' · no convergence in this window');
}

// ---------------- wind rose into an svg ----------------
// 16 sectors, wedge length = share of non-calm samples from that direction;
// calm (< 1 mph) reported as a %. Needle = current 10-minute mean direction,
// drawn in the text colour so it is not read as a second data series.
function drawRose(svg, r, opts){
 const {W,H,compact,needle}=opts, cx=W/2, cy=H/2, Rr=Math.min(W,H)/2-(compact?20:34);
 if(!r||r.error||!r.total){ svg.innerHTML=`<text class=ax x=${cx} y=${cy} text-anchor=middle>${(r&&r.error)||'no data yet'}</text>`; return ''; }
 const n=r.total-r.calm, share=r.sect.map(s=>n?s.n/n:0), mx=Math.max(...share,0.01);
 const ringMax=Math.ceil(mx*100/4)*4/100;
 const P=(a,rad)=>[cx+rad*Math.sin(a*Math.PI/180), cy-rad*Math.cos(a*Math.PI/180)];
 let g='';
 for(const f of (compact?[0.5,1]:[0.25,0.5,0.75,1])){ const rad=Rr*f;
   g+=`<circle class=gl cx="${cx}" cy="${cy}" r="${rad}" fill="none"/>`+
      `<text class=ax x=${cx+rad*0.707+3} y=${cy-rad*0.707-2}>${(ringMax*f*100).toFixed(0)}%</text>`; }
 for(let i=0;i<16;i+=2){ const a=i*22.5, [x1,y1]=P(a,Rr+3), [x2,y2]=P(a,Rr+(i%4?7:12));
   g+=`<line class=gl x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"/>`;
   if(i%4===0){ const [tx,ty]=P(a,Rr+(compact?13:24)); g+=`<text class=ax x=${tx} y=${ty+4} text-anchor=middle font-size=${compact?12:13}>${PTS16[i]}</text>`; } }
 for(let i=0;i<16;i++){ if(!r.sect[i].n) continue; const rad=Rr*share[i]/ringMax, a0=i*22.5-10.5, a1=i*22.5+10.5;
   const [ax,ay]=P(a0,rad), [bx,by]=P(a1,rad);
   g+=`<path class=wedge data-i=${i} fill="#3987e5" stroke="#1a1a1a" stroke-width="2" stroke-linejoin="round" d="M${cx},${cy} L${ax},${ay} A${rad},${rad} 0 0 1 ${bx},${by} Z"/>`; }
 const cur=needle;
 if(cur!=null){ const [nx,ny]=P(cur,Rr), [tx,ty]=P(cur,Rr-12), [lx,ly]=P(cur-6,Rr-22), [rx,ry]=P(cur+6,Rr-22);
   g+=`<line x1="${cx}" y1="${cy}" x2="${tx}" y2="${ty}" stroke="#e8e8e8" stroke-width="2" stroke-linecap="round"/>`+
      `<path fill="#e8e8e8" d="M${nx},${ny} L${lx},${ly} L${rx},${ry} Z"/>`; }
 g+=`<circle cx="${cx}" cy="${cy}" r="4" fill="#1a1a1a" stroke="#e8e8e8" stroke-width="2"/>`;
 svg.innerHTML=g;
 svg.querySelectorAll('.wedge').forEach(w=>{ w.onmousemove=e=>{ const i=+w.dataset.i, s=r.sect[i];
   showTip(e,`<b>${PTS16[i]}</b> (${i*22.5}°)<br>${(share[i]*100).toFixed(1)}% of the time<br>avg ${(s.sum/s.n).toFixed(1)} mph`); };
   w.onmouseleave=hideTip; });
 const best=share.indexOf(Math.max(...share));
 return `prevailing ${PTS16[best]} ${(share[best]*100).toFixed(0)}% · calm ${(r.calm/r.total*100).toFixed(0)}%`;
}

// ---------------- tabbed history section ----------------
const RANGES=[[6,'6 h'],[24,'24 h'],[72,'3 d'],[168,'7 d']];
function makeChart(root, tabs, key){
 const tabsEl=root.querySelector('.tabs:not(.ranges)'), rangesEl=root.querySelector('.ranges'),
       titleEl=root.querySelector('.title'), noteEl=root.querySelector('.note'), svg=root.querySelector('svg');
 let cur=null, hours=24;
 try{ cur=localStorage.getItem(key+'.tab'); hours=+localStorage.getItem(key+'.hours')||24; }catch(e){}
 const hp=new URLSearchParams(location.hash.slice(1));                  // deep link: #wx=temp&wxh=72
 if(hp.get(key)) cur=hp.get(key); if(hp.get(key+'h')) hours=+hp.get(key+'h');
 if(!tabs.some(t=>t.id===cur)) cur=tabs[0].id; if(!RANGES.some(r=>r[0]===hours)) hours=24;
 const save=()=>{ try{ localStorage.setItem(key+'.tab',cur); localStorage.setItem(key+'.hours',hours); }catch(e){} };
 function render(){
   tabsEl.innerHTML=tabs.map(t=>`<button class="${t.id===cur?'on':''}" data-id=${t.id}>${t.label}</button>`).join('');
   tabsEl.querySelectorAll('button').forEach(b=>b.onclick=()=>{ cur=b.dataset.id; save(); render(); draw(); });
   rangesEl.innerHTML=RANGES.map(([h,l])=>`<button class="${h===hours?'on':''}" data-h=${h}>${l}</button>`).join('');
   rangesEl.querySelectorAll('button').forEach(b=>b.onclick=()=>{ hours=+b.dataset.h; save(); render(); draw(); }); }
 async function draw(){
   const tab=tabs.find(t=>t.id===cur); titleEl.textContent=tab.title||tab.label; noteEl.textContent='';
   try{
     if(tab.kind==='rose'){ const m=latest&&latest.wind_dir_avg_10min_deg;
       noteEl.textContent=drawRose(svg, await fetchRose(hours), {W:800,H:300,compact:false,needle:m})+(m!=null?` · 10-min mean ${compass(m)} ${m}°`:''); return; }
     if(tab.kind==='xy'){ noteEl.textContent=drawScatter(svg, tab, await fetchXY(tab,hours)); return; }
     const d=await fetchSeries(tab,hours); drawLine(svg,tab,d,hours); if(d.note) noteEl.textContent=d.note;
   }catch(e){ svg.innerHTML=`<text class=ax x=400 y=150 text-anchor=middle>fetch failed</text>`; } }
 render(); draw(); setInterval(draw,60000);
}
makeChart(wxchart,[
 {id:'temp', label:'Temperature', metric:'davis_temperature_fahrenheit', unit:'°F', kind:'line', dec:1},
 {id:'hum',  label:'Humidity',    metric:'davis_humidity_percent', unit:'%', kind:'line', dec:0, from0:true, max:100},
 {id:'dew',  label:'Dew point',   metric:'davis_dew_point_fahrenheit', unit:'°F', kind:'line', dec:1},
 {id:'press',label:'Pressure',    title:'Barometric pressure, reduced to sea level', metric:'wxrx_env_pressure_sealevel_hpa', unit:'hPa', kind:'line', dec:1},
 {id:'rain', label:'Rain',        title:'Rain, accumulated', kind:'rainacc', unit:'in', dec:2, from0:true},
 {id:'rate', label:'Rain rate',   metric:'davis_rain_rate_inph', unit:'in/h', kind:'area', dec:2, from0:true},
 {id:'wind', label:'Wind',        title:'Wind, 10-minute average', metric:'davis_wind_speed_mph', avg:'10m', unit:'mph', kind:'area', dec:1, from0:true},
 {id:'dir',  label:'Direction',   title:'Wind rose', kind:'rose'},
],'wx');
makeChart(rfchart,[
 {id:'rate',   label:'Packets/min', metric:'packet_rate', title:'Good packets per minute', unit:'/min', kind:'area', dec:1, from0:true, max:30, step:10, ref:{v:23.4,label:'23.4/min = every slot heard = 100 %'}},
 {id:'rssi',   label:'RSSI',        metric:'davis_rssi_dbm', unit:'dBm', kind:'line', dec:0},
 {id:'age',    label:'Packet age',  metric:'davis_packet_age_seconds', unit:'s', kind:'line', dec:1, from0:true},
 {id:'bad',    label:'Bad/min',     metric:'bad_rate', title:'CRC failures per minute', unit:'/min', kind:'area', dec:2, from0:true},
 {id:'yield',  label:'Slot yield',  metric:'slot_yield', title:'Hop slot yield', unit:'%', kind:'line', dec:2},
 {id:'resync', label:'Resyncs',     metric:'resyncs', title:'Hop resyncs per 10 min', unit:'', kind:'area', dec:0, from0:true},
 {id:'streak', label:'Miss streak', metric:'davis_hop_miss_streak', title:'Consecutive missed slots', unit:'', kind:'line', dec:0, from0:true},
 {id:'solar',  label:'Solar panel', metric:'davis_solar_voltage_volts', title:'ISS solar panel voltage (msg 0x7, raw/300)', unit:'V', kind:'line', dec:2, from0:true},
 {id:'afc',    label:'AFC offset',  metric:'davis_afc_offset_khz', title:'Live tuning offset the AFC has walked to', unit:'kHz', kind:'line', dec:1},
 // The tempco curve, measured continuously. The AFC walks the offset to
 // wherever the transmitter actually is, so this IS the crystal drift - the
 // thing the overnight sweeps were run to find, now free and always on.
 // Expect roughly -0.15 to -0.4 kHz/degF, and NOT a straight line: AT-cut
 // crystals are cubic, so do not read a winter figure off a mild autumn slope.
 // Note the trace carries about +/-1.5 kHz of limit-cycle jitter from the
 // loop's discrete step - look at the trend, not any single point.
 {id:'drift',  label:'Drift vs temp', kind:'xy', title:'AFC offset against outdoor temperature (ISS crystal tempco)',
  ymetric:'davis_afc_offset_khz', xmetric:'davis_temperature_fahrenheit',
  xlabel:'Outdoor temperature (°F)', xunit:'°F', xdec:0,
  unit:'kHz', dec:1},
 // The cold-drift tab. Rate against OUTDOOR temperature, not against time: the
 // ISS transmitter walks off frequency as it cools, so packets that do arrive
 // are as strong as ever while more and more of them fail CRC. Pick 7 d to see
 // the knee properly - 6 h of one mild afternoon is a single blob.
 {id:'cold',   label:'Rate vs temp', kind:'xy', title:'Packet rate against outdoor temperature',
  ymetric:'packet_rate', xmetric:'davis_temperature_fahrenheit',
  xlabel:'Outdoor temperature (°F)', xunit:'°F', xdec:0,
  unit:'/min', dec:1, from0:true, max:24, step:4,
  ref:{v:23.4,label:'23.4/min = every slot heard'}},
],'rf');

// Compact rose in current conditions: the 24 h histogram is refetched every
// minute; the needle (always live - the direction in the latest packet) is
// redrawn on every 3 s tick. The 10-minute mean is on the Direction tab.
let roseHist=null;
function roseDraw(){
 if(!latest) return;
 const deg=latest.wind_direction_deg;
 const base=drawRose(rosesvg, roseHist, {W:250,H:270,compact:true,needle:deg});
 rosenote.textContent=(base?'rose, last 24 h: '+base:'')+(deg!=null?` · needle: live ${compass(deg)} ${deg}°`:'');
}
async function roseTick(){ try{ roseHist=await fetchRose(24); roseDraw(); }catch(e){} }
const tick0=tick; tick=async()=>{ await tick0(); roseDraw(); };
tick(); setInterval(tick,3000);
roseTick(); setInterval(roseTick,60000);
</script>"""


# davis_* = the ISS's own sensors; wxrx_* = our receiver box (RAK1906).
HIST_METRIC = re.compile(r"^(?:davis|wxrx)_[a-z0-9_]+$")
# Derived series the page may ask for by name. Kept server-side so the proxy
# never evaluates a caller-supplied expression.
HIST_PRESETS = {
    "packet_rate": "sum(rate(davis_packets_total_ok[5m])) * 60",
    "bad_rate":    "sum(rate(davis_packets_total_bad[5m])) * 60",
    "slot_yield":  "100 * (1 - sum(rate(davis_hop_missed_total[10m])) / sum(rate(davis_hop_slots_total[10m])))",
    "resyncs":     "sum(increase(davis_hop_resyncs_total[10m]))",
}

def history(path):
    """Proxy one davis_* series from Prometheus for the page's 24 h graphs."""
    import urllib.request, urllib.parse
    qs = urllib.parse.parse_qs(urllib.parse.urlsplit(path).query)
    metric = qs.get("metric", [""])[0]
    hours = min(168, max(1, int(qs.get("hours", ["24"])[0])))
    if not PROM_URL:
        return json.dumps({"error": "no history source (WXRX_PROM_URL unset)"})
    avg = qs.get("avg", [""])[0]                # e.g. 10m -> rolling mean over all samples
    if avg and not re.match(r"^[0-9]{1,3}m$", avg):
        return json.dumps({"error": "bad avg"})
    if metric in HIST_PRESETS:
        expr = HIST_PRESETS[metric]
    elif HIST_METRIC.match(metric):
        expr = f"max(avg_over_time({metric}[{avg}]))" if avg else f"max({metric})"
    else:
        return json.dumps({"error": "bad metric"})   # not an open proxy
    now = time.time()
    step = max(60, hours * 3600 // 720)        # ~720 points whatever the range
    # align to the step so two series fetched moments apart share timestamps
    # (the wind rose joins direction and speed on them)
    start = (now - hours * 3600) // step * step
    q = urllib.parse.urlencode({"query": expr, "start": start,
                                "end": now, "step": step})
    try:
        with urllib.request.urlopen(f"{PROM_URL}/api/v1/query_range?{q}", timeout=5) as r:
            res = json.load(r)["data"]["result"]
        vals = [[float(t), float(v)] for t, v in (res[0]["values"] if res else [])]
        return json.dumps({"metric": metric, "step": step, "values": vals})
    except Exception as e:
        return json.dumps({"error": f"history unavailable: {e}"})

class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path.startswith("/metrics"):
            body, ctype = metrics(), "text/plain; version=0.0.4"
        elif self.path.startswith("/api/current.json"):
            with lock:
                st = dict(state)
                st.update(rain_sums(time.time()))
                st.update(wind_rolling())
            st["dew_point_f"] = dew_point_f(st["temperature_f"], st["humidity_pct"])
            st["pressure_sealevel_hpa"] = sea_level_hpa(
                st["env_pressure_hpa"],
                f_to_c(st["temperature_f"]) if st["temperature_f"] is not None else st["env_temp_c"])
            if st["env_epoch"]:
                st["env_age_seconds"] = round(time.time() - st["env_epoch"], 1)
            if st["last_packet_epoch"]:
                st["packet_age_seconds"] = round(time.time() - st["last_packet_epoch"], 1)
            body, ctype = json.dumps(st, indent=1), "application/json"
        elif self.path.startswith("/api/history"):
            body, ctype = history(self.path), "application/json"
        elif self.path.startswith("/api/rain.json"):
            import urllib.parse
            qs = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
            hours = min(RAIN_LOG_DAYS * 24, max(1, int(qs.get("hours", ["24"])[0])))
            with lock:
                now = time.time()
                r = {"tip_inches": TIP_INCHES, "now": now, "hours": hours,
                     "events": [e for e in rain["events"] if e[0] >= now - hours * 3600]}
                r.update(rain_sums(now))
            body, ctype = json.dumps(r), "application/json"
        else:
            body, ctype = PAGE, "text/html; charset=utf-8"
        b = body.encode()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

if __name__ == "__main__":
    load_rain()
    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=rain_saver, daemon=True).start()
    print(f"wxrx: serving on :{PORT}", flush=True)
    Server(("", PORT), H).serve_forever()
