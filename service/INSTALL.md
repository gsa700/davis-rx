# wxrx install (on the Pi Zero 2W, hostname `wxrx`)

    sudo apt update && sudo apt install -y python3-serial
    sudo useradd --system --no-create-home --shell /usr/sbin/nologin wxrx
    sudo usermod -aG dialout wxrx
    sudo install -m755 wxrx.py /usr/local/bin/wxrx.py
    sudo install -m644 wxrx.service /etc/systemd/system/wxrx.service
    sudo systemctl daemon-reload && sudo systemctl enable --now wxrx
    systemctl status wxrx --no-pager

Check:  curl -s localhost:8000/metrics | head
        curl -s localhost:8000/api/current.json
Browser: http://wxrx:8000/  (or http://wxrx.example.lan:8000/)

## WiFi gotcha on Pi Zero 2W / Trixie
Creating a NEW NetworkManager connection fails with a key-mgmt error.
MODIFY the existing profile instead:
    nmcli connection modify <profile> wifi-sec.key-mgmt wpa-psk
    nmcli connection modify <profile> wifi-sec.psk '<passphrase>'

## USB gotcha
Only the INNER micro-USB port on a Zero 2W carries data. You need a micro-USB
OTG cable/adapter to reach the RAK's USB-C. The outer port is power only.

## Prometheus (on your Prometheus host, /etc/prometheus/prometheus.yml)
    - job_name: wxrx
      scrape_interval: 30s
      static_configs:
        - targets: ["wxrx.example.lan:8000"]
Then: sudo promtool check config /etc/prometheus/prometheus.yml && sudo systemctl reload prometheus

## Rain state (added 2026-09-03)
Totals and the tip log behind the 24 h graph persist in `/var/lib/wxrx/state.json`
(`StateDirectory=wxrx` in the unit; override with `WXRX_STATE=` for a dev run,
`WXRX_PORT=` to serve elsewhere, `WXRX_TIP_INCHES=0.00787` for a 0.2 mm metric bucket).
The page's 24 h graphs (one tab per metric) are proxied from Prometheus via `/api/history` -
set `WXRX_PROM_URL` in the unit (`Environment=WXRX_PROM_URL=http://<prometheus>:9090`); unset,
the tabs say "no history source" and only the rain tab (built from the local tip log) draws.
First install on a host that already has Prometheus history for this station: seed the file
from `davis_rain_counter` with `tools/rain-backfill.py` so the graph is not empty for a day:
    sudo systemctl stop wxrx
    sudo python3 tools/rain-backfill.py --prom http://<prometheus>:9090 --out /var/lib/wxrx/state.json
    sudo chown wxrx:dialout /var/lib/wxrx/state.json
    sudo systemctl start wxrx
Add `--dry-run` to see what it would write without touching the file.
