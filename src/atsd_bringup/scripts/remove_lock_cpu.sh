#!/usr/bin/env bash
# Restore the previous CPU ceiling before removing the obsolete power reservation.
set -euo pipefail
[[ $EUID -eq 0 ]]
if systemctl cat atsd-lock-power.service >/dev/null 2>&1; then
  systemctl stop atsd-lock-power.service
  systemctl disable atsd-lock-power.service 2>/dev/null || true
fi
if [[ -f /run/atsd-lock-power/cpu.json ]]; then
  /usr/bin/python3 -I /usr/local/libexec/atsd-lock-power.py restore
fi
rm -f /etc/systemd/system/atsd-lock-power.service \
  /etc/sudoers.d/atsd-lock-power /usr/local/libexec/atsd-lock-power.py
systemctl daemon-reload
