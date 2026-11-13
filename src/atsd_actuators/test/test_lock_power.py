"""Exercise actual node methods without requiring ROS or GPIO hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import uuid

import pytest

ROOT = Path(__file__).parents[1] / 'atsd_actuators'


@pytest.fixture
def rig():
    clock = NS(now=100.0)
    timeline = []
    def sleep(dt):
        clock.now += dt
    env = dict(Node=object, String=NS, Bool=NS, Float32=NS,
               time=NS(monotonic=lambda: clock.now, sleep=sleep),
               uuid=uuid, TRAVEL=('TO_DROP',), MODES=('solid',))
    for file in ('lock_node.py', 'light_node.py'):
        tree = ast.parse((ROOT / file).read_text())
        classes = [n for n in tree.body if isinstance(n, ast.ClassDef)]
        exec(compile(ast.Module(body=classes, type_ignores=[]), file, 'exec'), env)
    light = env['LightNode'].__new__(env['LightNode'])
    light.head = env['Channel'](18, .4, .5, 200, True)
    light.strip = env['Channel'](13, .4, .5, 200, True)
    light.head.dev = NS(value=.4)
    light.strip.dev = NS(value=.4)
    light.head.current = light.strip.current = .4
    light.power_until = 0
    light.phase = 0
    light.dt = .02
    light.dim_on_lock = True
    light.lock_busy = False
    light.head_level = light.strip_level = .4
    light.mode = 'solid'
    lock = env['LockNode'].__new__(env['LockNode'])
    lock.battery_ts = clock.now
    lock.battery = NS(present=True, percentage=.5, voltage=3.8)
    lock.guard = threading.Lock()
    lock.power_ready = threading.Event()
    lock.power_token = ''
    lock.busy = False
    lock.block_moving = True
    lock.mission_state = 'IDLE'
    lock.zone_policy = 'off'
    lock.last_pulse = 0
    lock.cooldown = 3
    lock.pulse_time = .4
    lock.get_logger = lambda: NS(info=lambda _: None)
    lock.pub_busy = NS(publish=lambda m: setattr(light, 'lock_busy', m.data))
    lock.pub_power = NS(publish=light.on_power_reserve)
    light.pub_ready = NS(publish=lock.on_power_ready)
    def on():
        light.tick()  # regular signal commands must not restore lights
        timeline.append(('on', clock.now, light.head.dev.value, light.strip.dev.value))
    lock.dev = NS(on=on, off=lambda: timeline.append(('off', clock.now)))
    return lock, light, clock, timeline


@pytest.mark.parametrize('pct,volts,present,age', [
    (.399, 3.8, True, 0), (.8, 3.49, True, 0),
    (float('nan'), 3.8, True, 0), (.8, float('nan'), True, 0),
    (.8, 3.8, False, 0), (.8, 3.8, True, 4), (1.1, 3.8, True, 0),
])
def test_battery_values_do_not_block_unlock(rig, pct, volts, present, age):
    lock, _, clock, timeline = rig
    lock.battery = NS(percentage=pct, voltage=volts, present=present)
    clock.now += age
    assert lock.on_unlock(None, NS()).success
    assert timeline[0][0] == 'on'


def test_lights_off_before_pulse_and_recover_after(rig):
    lock, light, clock, timeline = rig
    assert lock.on_unlock(None, NS()).success
    assert timeline[0] == ('on', 100.3, 0, 0)
    assert timeline[1][1] - timeline[0][1] == pytest.approx(.4)
    assert clock.now - timeline[1][1] == pytest.approx(1)
    assert not lock.busy
    light.tick()
    assert 0 < light.head.current < .4


def test_missing_or_wrong_ack_cancels(rig):
    lock, light, _, timeline = rig
    light.pub_ready.publish = lambda _: lock.on_power_ready(NS(data='old-token'))
    assert not lock.on_unlock(None, NS()).success
    assert all(t[0] != 'on' for t in timeline)
    assert not lock.busy


def test_gpio_exception_releases_reservation(rig):
    lock, light, _, timeline = rig
    def fail():
        raise OSError('GPIO failed')
    lock.dev.on = fail
    assert not lock.on_unlock(None, NS()).success
    assert timeline[-1][0] == 'off'
    assert not lock.busy and light.power_until == 0


def test_battery_drop_does_not_cancel(rig):
    lock, light, _, timeline = rig
    def ack(msg):
        lock.battery.percentage = .3
        lock.on_power_ready(msg)
    light.pub_ready.publish = ack
    assert lock.on_unlock(None, NS()).success
    assert timeline[0][0] == 'on'


def test_no_gpio_does_not_report_open(rig):
    lock, _, _, timeline = rig
    lock.dev = None
    assert not lock.on_unlock(None, NS()).success
    assert not timeline


def test_motion_during_preparation_cancels(rig):
    lock, light, _, timeline = rig
    def ack(msg):
        lock.mission_state = 'TO_DROP'
        lock.on_power_ready(msg)
    light.pub_ready.publish = ack
    assert not lock.on_unlock(None, NS()).success
    assert all(t[0] != 'on' for t in timeline)


def test_reservation_expires_if_lock_process_disappears(rig):
    _, light, clock, _ = rig
    light.on_power_reserve(NS(data='test'))
    light.tick()
    assert light.head.current == 0
    clock.now += 6
    light.tick()
    assert light.head.current > 0


def test_no_battery_telemetry_required(rig):
    lock, _, _, timeline = rig
    del lock.battery
    del lock.battery_ts
    assert lock.on_unlock(None, NS()).success
    assert timeline[0][0] == 'on'
