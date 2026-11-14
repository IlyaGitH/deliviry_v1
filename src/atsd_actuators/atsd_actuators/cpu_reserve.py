"""Start a bounded, independently restored CPU reservation through systemd."""
import subprocess


def cpu_reserve(action):
    if action not in ('restart', 'stop'):
        raise ValueError('Invalid CPU reservation action')
    result = subprocess.run(
        ['/usr/bin/sudo', '-n', '/usr/bin/systemctl', action, 'atsd-lock-power.service'],
        capture_output=True, text=True, timeout=8.0, check=False)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or 'CPU reservation failed')
