import pytest

from atsd_actuators.pins import RESERVED, check_pin, check_unique, BUZZER, LOCK, HEADLIGHT, STRIP, DOOR_SENSOR
from atsd_actuators.signal_logic import Inputs, SignalLogic


def test_pinout_has_no_conflicts():
    check_unique({'фары': HEADLIGHT, 'лента': STRIP, 'замок': LOCK, 'зуммер': BUZZER, 'геркон': DOOR_SENSOR})
    for pin in (HEADLIGHT, STRIP, LOCK, BUZZER, DOOR_SENSOR):
        check_pin(pin, 'x')
    for pin in RESERVED:
        with pytest.raises(ValueError):
            check_pin(pin, 'x')


def test_signal_rules():
    L = SignalLogic()
    moving = Inputs(link=True, state='TO_DROP')
    o = L.decide(moving, 0.0)
    assert (o.mode, o.buzzer, o.head) == ('beacon', 'off', 1.0)
    assert L.decide(Inputs(link=True, state='TO_DROP', obstacle_m=1.2), 0).buzzer == 'approach'
    assert L.decide(Inputs(link=True, state='TO_DROP', pause=['red_light']), 0).mode == 'blink'
    assert L.decide(Inputs(link=True, state='TO_DROP', flags=['recovery']), 0).buzzer == 'reverse'
    assert L.decide(Inputs(link=True, state='WAIT_LOAD'), 0).mode == 'solid'
    L.on_event('CROSSWALK_SLOW', 10.0)
    assert L.decide(moving, 12.0).buzzer == 'approach'
    assert L.decide(moving, 20.0).buzzer == 'off'


def test_alarm_times_out_and_oneshot_blocks():
    L = SignalLogic()
    assert L.decide(Inputs(link=True, state='FAULT'), 0.0).buzzer == 'alarm'
    assert L.decide(Inputs(link=True, state='FAULT'), 7.0).buzzer == 'off'
    assert L.on_event('ARRIVED_DROP', 100.0) == 'arrive'
    assert not L.buzzer_free(100.5) and L.buzzer_free(101.3)
