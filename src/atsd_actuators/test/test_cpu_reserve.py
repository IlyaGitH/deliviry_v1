import importlib.util
from pathlib import Path

import pytest

HELPER = Path(__file__).parents[2] / 'atsd_bringup/systemd/lock-power.py'


@pytest.fixture
def helper(tmp_path):
    spec = importlib.util.spec_from_file_location('lock_power_helper', HELPER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.POLICIES = tmp_path / 'cpufreq'
    mod.STATE = tmp_path / 'cpu.json'
    for name, maximum in [('policy0', 2400000), ('policy1', 1800000)]:
        p = mod.POLICIES / name
        p.mkdir(parents=True)
        (p / 'scaling_max_freq').write_text(str(maximum))
        (p / 'cpuinfo_min_freq').write_text('1500000')
    return mod


def test_restore_preserves_individual_previous_limits(helper):
    helper.reserve()
    assert all(p.read_text() == '1500000' for p in helper.POLICIES.glob('*/scaling_max_freq'))
    helper.restore()
    assert (helper.POLICIES / 'policy0/scaling_max_freq').read_text() == '2400000'
    assert (helper.POLICIES / 'policy1/scaling_max_freq').read_text() == '1800000'
    assert not helper.STATE.exists()
    helper.restore()  # harmless when no reservation exists


def test_snapshot_survives_partial_reserve_failure(helper):
    (helper.POLICIES / 'policy1/cpuinfo_min_freq').unlink()
    with pytest.raises(OSError):
        helper.reserve()
    assert helper.STATE.exists()
    helper.restore()
    assert (helper.POLICIES / 'policy0/scaling_max_freq').read_text() == '2400000'


def test_restore_in_separate_process_context(helper):
    helper.reserve()
    # Load a new module just as ExecStopPost starts a separate interpreter.
    spec = importlib.util.spec_from_file_location('fresh_helper', HELPER)
    fresh = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fresh)
    fresh.STATE, fresh.POLICIES = helper.STATE, helper.POLICIES
    fresh.restore()
    assert (helper.POLICIES / 'policy0/scaling_max_freq').read_text() == '2400000'


def test_unavailable_cpufreq_fails_before_snapshot(helper):
    helper.POLICIES = helper.POLICIES / 'missing'
    with pytest.raises(RuntimeError):
        helper.reserve()
    assert not helper.STATE.exists()
