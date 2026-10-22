"""AnalogDiscoveryScope against a fake libdwf that records every call."""

import ctypes

import numpy as np
import pytest
from analog_discovery import dwf as d
from analog_discovery.scope import AnalogDiscoveryScope, CaptureSettings


def _value(arg):
    return arg.value if hasattr(arg, 'value') else arg


class FakeLib:
    """Two AD3s (the second in use) and an unrelated device; config 1 has the largest buffer."""

    def __init__(self, devices=None, buffers=(16384, 32768, 8192), range_pp_v=None):
        self.devices = devices or [
            (d.DEVID_DISCOVERY3, '210415BF81F3', False),
            (99, 'OTHERDEVICE0', False),
            (d.DEVID_DISCOVERY3, '210415BF0000', True),
        ]
        self.buffers = buffers
        self.range_pp_v = range_pp_v  # None: the device takes what is asked
        self.calls = []
        self.fail = set()
        self.state = d.STATE_ARMED
        self.position_s = None
        self.ranges = {}
        self.frequency = None
        self.buffer = None

    def __getattr__(self, name):
        if not name.startswith('FDwf'):
            raise AttributeError(name)

        def f(*args):
            self.calls.append((name, [_value(a) for a in args
                                      if not isinstance(a, (ctypes._Pointer, ctypes.Array))]))
            if name in self.fail:
                return 0
            handler = getattr(self, '_' + name, None)
            if handler:
                handler(*args)
            return 1
        return f

    def _FDwfGetLastErrorMsg(self, buf):
        buf.value = b'fake failure'

    def _FDwfEnum(self, _filter, count):
        count.contents.value = len(self.devices)

    def _FDwfEnumDeviceType(self, i, devid, devver):
        devid.contents.value = self.devices[i.value][0]

    def _FDwfEnumDeviceIsOpened(self, i, used):
        used.contents.value = int(self.devices[i.value][2])

    def _FDwfEnumSN(self, i, buf):
        buf.value = self.devices[i.value][1].encode()

    def _FDwfEnumConfig(self, _i, count):
        count.contents.value = len(self.buffers)

    def _FDwfEnumConfigInfo(self, cfg, _info, value):
        value.contents.value = self.buffers[cfg.value]

    def _FDwfDeviceConfigOpen(self, i, cfg, hdwf):
        self.opened = (i.value, cfg.value)
        hdwf.contents.value = 7

    def _FDwfAnalogInChannelRangeSet(self, _h, ch, volts):
        self.ranges[ch.value] = self.range_pp_v or volts.value

    def _FDwfAnalogInChannelRangeGet(self, _h, ch, volts):
        volts.contents.value = self.ranges[ch.value]

    def _FDwfAnalogInFrequencySet(self, _h, hz):
        self.frequency = hz.value

    def _FDwfAnalogInFrequencyGet(self, _h, hz):
        hz.contents.value = self.frequency

    def _FDwfAnalogInBufferSizeSet(self, _h, n):
        self.buffer = n.value

    def _FDwfAnalogInBufferSizeGet(self, _h, n):
        n.contents.value = self.buffer

    def _FDwfAnalogInTriggerPositionSet(self, _h, s):
        self.position_s = s.value

    def _FDwfAnalogInTriggerPositionStatus(self, _h, s):
        s.contents.value = self.position_s

    def _FDwfAnalogInStatus(self, _h, _read, state):
        state.contents.value = self.state

    def _FDwfAnalogInStatusData(self, _h, ch, buf, n):
        for i in range(n.value):
            buf[i] = ch.value + i * 1e-3

    def _FDwfAnalogInStatusAutoTriggered(self, _h, auto):
        auto.contents.value = 0

    def _FDwfAnalogInStatusTime(self, _h, sec, tick, tps):
        sec.contents.value, tick.contents.value, tps.contents.value = 1792, 12345, 100000000


def make(lib=None, **kwargs):
    lib = lib or FakeLib()
    return AnalogDiscoveryScope(dwf=d.Dwf(lib), **kwargs), lib


def test_list_devices_keeps_only_analog_discovery():
    scope, _ = make()
    assert [(x['serial'], x['in_use']) for x in scope.list_devices()] == [
        ('210415BF81F3', False), ('210415BF0000', True)]


def test_open_takes_the_first_free_unit_and_the_largest_buffer():
    scope, lib = make()
    scope.open()
    assert lib.opened == (0, 1)
    assert scope.max_buffer_samples == 32768
    assert scope.serial == '210415BF81F3'


def test_open_refuses_a_unit_in_use_and_unknown_serials():
    scope, _ = make(serial='210415BF0000')
    with pytest.raises(d.DwfError, match='another process'):
        scope.open()
    scope, _ = make(serial='NOPE')
    with pytest.raises(d.DwfError, match='no free'):
        scope.open()


def test_open_with_an_explicit_config():
    scope, lib = make(config_index=2)
    scope.open()
    assert lib.opened == (0, 2) and scope.max_buffer_samples == 8192
    scope, _ = make(config_index=5)
    with pytest.raises(ValueError):
        scope.open()


def configured(settings=None, lib=None):
    scope, lib = make(lib)
    scope.open()
    rate = scope.configure(settings or CaptureSettings())
    return scope, lib, rate


def calls_named(lib, name):
    return [args for n, args in lib.calls if n == name]


def test_configure_sends_peak_to_peak_ranges_and_edge_trigger():
    s = CaptureSettings(sample_rate_hz=2e6, buffer_samples=4096, ch1_range_peak_v=25.0,
                        trigger_source='ch2', trigger_slope='falling', trigger_level_v=-0.3,
                        trigger_hysteresis_v=0.02, pretrigger_samples=1000, auto_timeout_s=0.5)
    scope, lib, rate = configured(s)
    assert rate == 2e6
    assert calls_named(lib, 'FDwfAnalogInChannelRangeSet') == [[7, 0, 50.0], [7, 1, 5.0]]
    assert calls_named(lib, 'FDwfAnalogInTriggerSourceSet') == [[7, d.TRIGSRC_DETECTOR_ANALOG_IN]]
    assert calls_named(lib, 'FDwfAnalogInTriggerChannelSet') == [[7, 1]]
    assert calls_named(lib, 'FDwfAnalogInTriggerConditionSet') == [[7, 1]]
    assert calls_named(lib, 'FDwfAnalogInTriggerLevelSet') == [[7, -0.3]]
    assert calls_named(lib, 'FDwfAnalogInTriggerAutoTimeoutSet') == [[7, 0.5]]
    assert calls_named(lib, 'FDwfAnalogInAcquisitionModeSet') == [[7, d.ACQMODE_SINGLE]]
    # (4096 / 2 - 1000) samples at 2 MHz
    assert lib.position_s == pytest.approx(524e-6)


def test_configure_disables_ch2_and_free_runs():
    s = CaptureSettings(ch2_enabled=False, trigger_source='none', auto_timeout_s=0.1)
    _, lib, _ = configured(s)
    assert calls_named(lib, 'FDwfAnalogInChannelEnableSet') == [[7, 0, 1], [7, 1, 0]]
    assert calls_named(lib, 'FDwfAnalogInTriggerSourceSet') == [[7, d.TRIGSRC_NONE]]
    assert calls_named(lib, 'FDwfAnalogInTriggerLevelSet') == []


def test_configure_rejects_a_range_narrower_than_asked():
    with pytest.raises(ValueError, match='narrower'):
        configured(CaptureSettings(ch1_range_peak_v=25.0), FakeLib(range_pp_v=5.0))


def test_configure_rejects_a_buffer_larger_than_the_device():
    with pytest.raises(ValueError, match='exceeds'):
        configured(CaptureSettings(buffer_samples=65536, pretrigger_samples=0))


def test_a_failing_call_raises_with_the_sdk_message():
    lib = FakeLib()
    lib.fail.add('FDwfAnalogInFrequencySet')
    with pytest.raises(d.DwfError, match='FDwfAnalogInFrequencySet failed: fake failure'):
        configured(lib=lib)


def test_poll_returns_nothing_until_done_then_the_capture():
    s = CaptureSettings(buffer_samples=100, pretrigger_samples=30)
    scope, lib, _ = configured(s)
    scope.arm()
    assert calls_named(lib, 'FDwfAnalogInConfigure') == [[7, 1, 1]]
    assert scope.poll() is None
    lib.state = d.STATE_DONE
    cap = scope.poll()
    assert cap.trigger_index == 30
    assert len(cap.ch1_v) == len(cap.ch2_v) == 100
    np.testing.assert_allclose(cap.ch2_v[:3], [1.0, 1.001, 1.002])
    assert cap.device_time == (1792, 12345, 100000000) and not cap.auto_triggered


@pytest.mark.parametrize('kwargs', [
    {'sample_rate_hz': 0.0},
    {'sample_rate_hz': 200e6},
    {'buffer_samples': 1},
    {'pretrigger_samples': 8192},
    {'trigger_source': 'ch3'},
    {'trigger_source': 'ch2', 'ch2_enabled': False},
    {'trigger_level_v': 2.5},             # at the +-2.5 V range edge
    {'trigger_slope': 'up'},
    {'trigger_hysteresis_v': -0.1},
    {'auto_timeout_s': -1.0},
    {'trigger_source': 'none'},           # free running without auto timeout
    {'ch1_range_peak_v': 0.0},
])
def test_settings_validation(kwargs):
    with pytest.raises(ValueError):
        CaptureSettings(**kwargs).validate()


def test_close_is_safe_twice():
    scope, lib = make()
    scope.open()
    scope.close()
    scope.close()
    assert len(calls_named(lib, 'FDwfDeviceClose')) == 1
