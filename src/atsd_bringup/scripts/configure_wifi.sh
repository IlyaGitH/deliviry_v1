#!/usr/bin/env bash
# Change the next boot without dropping the current SSH connection.
set -euo pipefail
restore_profiles=0
if grep -Eq '^[[:space:]]*(export[[:space:]]+)?AP_ONLY=1([[:space:]#]|$)' /etc/atsd/wifi.conf; then restore_profiles=1; fi
python3 - <<'PY'
from pathlib import Path
import re
p=Path('/etc/atsd/wifi.conf')
if not p.exists():
    raise SystemExit('Нет /etc/atsd/wifi.conf: сначала выполните install.sh')
s=p.read_text()
for key,value in [('AP_ONLY','0'),('AP_RETRY_ENABLED','0'),('WAIT_S','20'),('STABLE_S','5')]:
    s=re.sub(r'(?m)^\s*(?:export\s+)?'+key+r'=.*\n?', '', s)
s=s.rstrip()+'\nAP_ONLY=0\nAP_RETRY_ENABLED=0\nWAIT_S=20\nSTABLE_S=5\n'
p.write_text(s)
p.chmod(0o600)
PY
# Undo profile flags from the previous AP-only update, only when leaving that mode.
(( restore_profiles )) || exit 0
# Updating autoconnect does not disconnect the currently active station.
profiles=$(nmcli --wait 10 -t -f UUID,TYPE connection show)
while IFS=: read -r uuid type; do
  [[ "$type" == '802-11-wireless' || "$type" == 'wifi' ]] || continue
  mode=$(nmcli --wait 10 -g 802-11-wireless.mode connection show uuid "$uuid")
  [[ "$mode" == ap ]] && continue
  nmcli --wait 10 connection modify uuid "$uuid" connection.autoconnect yes
done <<< "$profiles"
