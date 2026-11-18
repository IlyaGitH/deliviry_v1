#!/usr/bin/env bash
# Apply the requested detection distance even when the Pi keeps a local YAML.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
import re
p=Path('/opt/atsd/ws/src/atsd_bringup/config/atsd.yaml')
s=p.read_text()
s,n=re.subn(r'(?m)^(    look_ahead:) .*$',r'\1 1.0  # distance from front edge, metres',s)
if n != 1: raise SystemExit('Expected exactly one obstacle look_ahead in atsd.yaml')
p.write_text(s)
PY
