#!/usr/bin/env python3
# davis-rx — Davis Vantage Pro2 ISS receiver
# Copyright (C) 2026  davis-rx contributors
# Licensed under the GNU General Public License v3.0 or later.
# This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.
"""Log the RFM69 receiver's frequency-error readings to CSV.

The point of the RFM69 build is that it measures the transmitter's frequency
error IN HARDWARE, per packet, in ~61 Hz steps -- against the SX1262 software
loop's 1500 Hz steps inferred over a 20-minute cycle. That should be good enough
to see the crystal's cubic temperature curvature directly instead of inferring
it from slopes that disagree by a factor of three.

Needs nothing but pyserial; no database, no Prometheus. Run it next to the
receiver and let it fill a file.

    ./fei-log.py /dev/ttyACM0            (Linux)
    ./fei-log.py COM5                    (Windows)
"""
import csv, re, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
OUT  = sys.argv[2] if len(sys.argv) > 2 else "fei-log.csv"
BAUD = 115200

# Packet lines carry the per-packet datum; STATUS lines carry the rolling summary.
PKT = re.compile(r"\[CRC (OK|BAD)\s*\].*?msg=0x([0-9A-Fa-f]).*?rssi=(-?\d+).*?"
                 r"ch=(\d+).*?fei=(-?\d+).*?afc=(-?\d+)")
STS = re.compile(r"fei_mean=(-?\d+)\s+fei_min=(-?\d+)\s+fei_max=(-?\d+)\s+fei_n=(\d+)")

def main():
    ser = serial.Serial(PORT, BAUD, timeout=2)
    print(f"reading {PORT} -> {OUT}   (ctrl-C to stop)")
    n = 0
    with open(OUT, "a", newline="") as fh:
        w = csv.writer(fh)
        if fh.tell() == 0:
            w.writerow(["epoch", "iso", "kind", "crc", "msgtype", "rssi_dbm",
                        "channel", "fei_hz", "afc_hz",
                        "fei_mean_hz", "fei_min_hz", "fei_max_hz", "fei_n"])
        while True:
            try:
                line = ser.readline().decode("ascii", "replace").strip()
            except serial.SerialException as e:
                print("serial error:", e); time.sleep(2); continue
            if not line:
                continue
            now = time.time(); iso = time.strftime("%Y-%m-%dT%H:%M:%S")

            m = PKT.search(line)
            if m:
                crc, msg, rssi, ch, fei, afc = m.groups()
                w.writerow([round(now, 1), iso, "packet", crc, msg, rssi, ch,
                            fei, afc, "", "", "", ""])
                fh.flush(); n += 1
                if n % 20 == 0:
                    print(f"  {n} packets, last fei={fei} Hz afc={afc} Hz "
                          f"rssi={rssi} ch={ch}")
                continue

            s = STS.search(line)
            if s:
                mean, lo, hi, cnt = s.groups()
                w.writerow([round(now, 1), iso, "status", "", "", "", "",
                            "", "", mean, lo, hi, cnt])
                fh.flush()
                # The SPREAD is the headline: it is this chip's measurement noise,
                # and the number to compare against the software loop's 1500 Hz step.
                print(f"  STATUS mean={mean} Hz  spread={int(hi)-int(lo)} Hz  n={cnt}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
