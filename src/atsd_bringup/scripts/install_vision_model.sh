#!/usr/bin/env bash
# Bundled data only: never execute Python files supplied with the model archive.
set -euo pipefail
model=/opt/atsd/ws/src/atsd_bringup/models/signs_ncnn_model
for file in model.ncnn.param model.ncnn.bin metadata.yaml; do
  test -s "$model/$file"
done
python3 - <<'PY'
from pathlib import Path
import re
p=Path('/opt/atsd/ws/src/atsd_bringup/config/atsd.yaml')
s=p.read_text()
s,n=re.subn(r'(?m)^(    model_path:) .*$',r'\1 /opt/atsd/ws/src/atsd_bringup/models/signs_ncnn_model',s)
if n != 1: raise SystemExit('Expected exactly one vision model_path in atsd.yaml')
p.write_text(s)
PY
