import os
from pathlib import Path
import subprocess

SCRIPT = Path(__file__).resolve().parents[1] / 'systemd/wifi.sh'


def run(body):
    result = subprocess.run(['bash', '-c', 'source "$1"\n' + body, 'test', str(SCRIPT)],
                            env={**os.environ, 'ATSD_WIFI_CONF': '/nonexistent'},
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    return result.stdout


def test_failed_station_query_does_not_mean_no_clients():
    assert 'KEEP' in run('iw() { return 1; }; no_clients || echo KEEP')
    assert 'EMPTY' in run('iw() { return 0; }; no_clients && echo EMPTY')


def test_late_interface_and_unstable_connection_eventually_start_ap():
    out = run('''
      t=1; WAIT_S=3; STABLE_S=30; POLL_S=1
      date() { echo "$t"; }
      sleep() { t=$((t+1)); if ((t>10)); then exit 0; fi; }
      nmcli() { if [[ "$*" == '-g GENERAL.STATE dev show wlan0' ]] && ((t<3)); then return 1; fi; return 0; }
      dev_state() { echo '100 (connected)'; }
      dev_conn() { echo home; }
      signal() { echo 80; }
      start_ap() { echo AP_STARTED; exit 0; }
      main
    ''')
    assert 'Ожидаю интерфейс' in out
    assert 'AP_STARTED' in out


def test_running_ap_stays_up_by_default():
    out = run('''
      t=1000; RETRY_S=1
      date() { echo "$t"; }
      sleep() { t=$((t+1)); if ((t>1003)); then exit 0; fi; }
      nmcli() { if [[ "$*" == 'con down atsd-hotspot' ]]; then echo DROPPED; fi; return 0; }
      dev_state() { echo '100 (connected)'; }
      dev_conn() { echo atsd-hotspot; }
      iw() { return 0; }
      main
    ''')
    assert 'DROPPED' not in out


def test_unknown_signal_is_not_a_stable_station():
    assert 'UNSTABLE' in run("dev_state() { echo '100 (connected)'; }; signal() { echo '--'; }; link_ok || echo UNSTABLE")


def test_stable_known_network_keeps_connection():
    out = run('''
      t=1; WAIT_S=6; STABLE_S=2
      date() { echo "$t"; }
      sleep() { t=$((t+1)); if ((t>12)); then exit 0; fi; }
      nmcli() { return 0; }
      dev_state() { echo '100 (connected)'; }
      dev_conn() { echo home; }
      signal() { echo 80; }
      start_ap() { echo AP_STARTED; exit 0; }
      main
    ''')
    assert 'AP_STARTED' not in out


def test_ap_auto_channel_avoids_nmcli_zero_channel_rejection():
    out = run('''
      AP_PASS=test-password; activated=0
      nmcli() {
        if [[ "$1 $2" == 'con add' ]]; then
          while (($#)); do
            if [[ "$1" == 802-11-wireless.channel ]]; then
              echo 'Unexpected explicit channel' >&2
              return 1
            fi
            shift
          done
        elif [[ "$1 $2" == 'con up' ]]; then
          activated=1
        fi
        return 0
      }
      start_ap || exit 1
      [[ $activated == 1 ]] || exit 1
      echo SUCCESS
    ''')
    assert 'SUCCESS' in out


def test_ap_fixed_channel_is_preserved():
    out = run('''
      AP_PASS=test-password; AP_CHANNEL=6
      found=0; activated=0
      nmcli() {
        if [[ "$1 $2" == 'con add' ]]; then
          while (($#)); do
            if [[ "$1" == 802-11-wireless.channel && "$2" == 6 ]]; then
              found=1
            fi
            shift
          done
        elif [[ "$1 $2" == 'con up' ]]; then
          activated=1
        fi
        return 0
      }
      start_ap || exit 1
      [[ $found == 1 && $activated == 1 ]] || exit 1
      echo SUCCESS
    ''')
    assert 'SUCCESS' in out
