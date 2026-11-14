#!/usr/bin/python3
"""Root-only CPU reservation; systemd always calls restore after hold exits."""
import json
import os
from pathlib import Path
import socket
import sys
import time

POLICIES = Path('/sys/devices/system/cpu/cpufreq')
STATE = Path('/run/atsd-lock-power/cpu.json')


def restore():
    if not STATE.exists():
        return
    saved = json.loads(STATE.read_text())
    errors = []
    for name, previous in saved.items():
        try:
            (POLICIES / name / 'scaling_max_freq').write_text(str(int(previous)))
        except OSError as exc:
            errors.append(str(exc))
    if errors:
        raise RuntimeError('; '.join(errors))
    STATE.unlink()


def reserve():
    policies = sorted(POLICIES.glob('policy*'))
    if not policies:
        raise RuntimeError('CPU frequency control unavailable')
    saved = {p.name: int((p / 'scaling_max_freq').read_text()) for p in policies}
    # Persist the entire snapshot before touching any policy, including on partial failure.
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(saved))
    temporary.replace(STATE)
    for p in policies:
        minimum = int((p / 'cpuinfo_min_freq').read_text())
        (p / 'scaling_max_freq').write_text(str(minimum))
        if int((p / 'scaling_max_freq').read_text()) != minimum:
            raise RuntimeError('CPU limit did not apply')
        print(f'{p.name}: CPU maximum {minimum} kHz', flush=True)


def hold():
    reserve()
    time.sleep(2.0)
    address = os.environ['NOTIFY_SOCKET']
    if address.startswith('@'):
        address = '\0' + address[1:]
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
        sock.connect(address)
        sock.sendall(b'READY=1')
    # Independent of ROS/SSH. ExecStopPost also runs on failure or SIGKILL.
    time.sleep(15.0)


if __name__ == '__main__':
    if sys.argv[1:] == ['hold']:
        hold()
    elif sys.argv[1:] == ['restore']:
        restore()
    else:
        raise SystemExit('Expected hold or restore')
