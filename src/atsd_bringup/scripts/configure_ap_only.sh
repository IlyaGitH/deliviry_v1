#!/usr/bin/env bash
# Change the next boot without dropping the current SSH connection.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
import re
p=Path('/etc/atsd/wifi.conf')
if not p.exists():
    raise SystemExit('Нет /etc/atsd/wifi.conf: сначала выполните install.sh')
s=p.read_text()
for key,value in [('AP_ONLY','1'),('AP_RETRY_ENABLED','0')]:
    s=re.sub(r'(?m)^\s*(?:export\s+)?'+key+r'=.*\n?', '', s)
s=s.rstrip()+'\nAP_ONLY=1\nAP_RETRY_ENABLED=0\n'
p.write_text(s)
p.chmod(0o600)
PY
# Persistent profile flags stop NetworkManager racing the AP service on boot.
# Updating autoconnect does not disconnect the currently active station.
profiles=$(nmcli --wait 10 -t -f UUID,TYPE connection show)
while IFS=: read -r uuid type; do
  [[ "$type" == '802-11-wireless' || "$type" == 'wifi' ]] || continue
  mode=$(nmcli --wait 10 -g 802-11-wireless.mode connection show uuid "$uuid")
  [[ "$mode" == ap ]] && continue
  nmcli --wait 10 connection modify uuid "$uuid" connection.autoconnect no
done <<< "$profiles"
