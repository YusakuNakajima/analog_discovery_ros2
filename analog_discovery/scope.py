"""Triggered oscilloscope captures from an Analog Discovery (AD3, also AD2).

One capture = one buffer of buffer_samples per channel around a trigger
(acqmodeSingle). After each capture is read the instrument is re-armed, so
events that happen while a capture is being read out are missed (dead time).

Units: V, Hz, s throughout. The SDK's range setting is peak-to-peak; it is
converted here from the +-peak value the user gives.
"""

import ctypes
import dataclasses
import math
import time

import numpy as np

from analog_discovery import dwf as d

MAX_SAMPLE_RATE_HZ = 125e6  # AD3 (AD2: 100 MS/s)
# Wait after configure() before the first arm. On the AD3 at OnoLab (2026-10-22,
# nothing on CH1, +-2.5 V range, ch1 rising 0.05 V) the first configure+arm after
# opening the device triggered on a transient 47-64 ms after arming (0.043-0.072 V)
# in 3 of the 4 processes where it was looked at; later configures did not. With
# a 0.2 s wait: 0 of 8 (first cycle included). Provisional: about 3x the longest
# transient seen.
DEFAULT_SETTLE_S = 0.2


@dataclasses.dataclass
class CaptureSettings:
    # Defaults match config/analog_discovery.yaml (AE9501B RF output; see there).
    sample_rate_hz: float = 10e6
    buffer_samples: int = 32768
    ch2_enabled: bool = True
    ch1_range_peak_v: float = 2.5  # +- V; AD3 has +-2.5 V and +-25 V
    ch2_range_peak_v: float = 2.5
    trigger_source: str = 'ch1'  # 'ch1' | 'ch2' | 'none' (free running, needs auto timeout)
    trigger_slope: str = 'rising'  # 'rising' | 'falling' | 'either'
    trigger_level_v: float = 0.02
    trigger_hysteresis_v: float = 0.005
    pretrigger_samples: int = 4096  # samples before the trigger in each capture
    auto_timeout_s: float = 0.0  # 0: wait for a real trigger; > 0: capture anyway after this

    def validate(self):
        """Check what can be checked without the device; raise ValueError."""
        for name in ('sample_rate_hz', 'ch1_range_peak_v', 'ch2_range_peak_v',
                     'trigger_level_v', 'trigger_hysteresis_v', 'auto_timeout_s'):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f'{name} must be finite')
        if not 0.0 < self.sample_rate_hz <= MAX_SAMPLE_RATE_HZ:
            raise ValueError(f'sample_rate_hz must be in (0, {MAX_SAMPLE_RATE_HZ:g}]')
        if self.buffer_samples < 2:
            raise ValueError('buffer_samples must be at least 2')
        if not 0 <= self.pretrigger_samples < self.buffer_samples:
            raise ValueError('pretrigger_samples must be in [0, buffer_samples)')
        for name in ('ch1_range_peak_v', 'ch2_range_peak_v'):
            if not getattr(self, name) > 0.0:
                raise ValueError(f'{name} must be positive')
        if self.trigger_source not in ('ch1', 'ch2', 'none'):
            raise ValueError("trigger_source must be 'ch1', 'ch2' or 'none'")
        if self.trigger_source == 'ch2' and not self.ch2_enabled:
            raise ValueError('trigger_source ch2 needs ch2_enabled')
        if self.trigger_slope not in d.TRIGGER_SLOPES:
            raise ValueError(f'trigger_slope must be one of {sorted(d.TRIGGER_SLOPES)}')
        if self.trigger_source != 'none':
            range_v = (self.ch1_range_peak_v if self.trigger_source == 'ch1'
                       else self.ch2_range_peak_v)
            if abs(self.trigger_level_v) >= range_v:
                raise ValueError(
                    f'trigger_level_v {self.trigger_level_v} V is outside the '
                    f'+-{range_v} V range of {self.trigger_source}')
        if self.trigger_hysteresis_v < 0.0:
            raise ValueError('trigger_hysteresis_v must not be negative')
        if self.auto_timeout_s < 0.0:
            raise ValueError('auto_timeout_s must not be negative')
        if self.trigger_source == 'none' and self.auto_timeout_s == 0.0:
            raise ValueError("trigger_source 'none' needs auto_timeout_s > 0")

    def trigger_position_s(self):
        """FDwfAnalogInTriggerPositionSet value that puts the trigger at pretrigger_samples.

        Assumes the SDK convention that position 0 is the buffer centre and a
        positive position moves the trigger towards the start of the buffer.
        UNVERIFIED on hardware: check with ad_selftest.py.
        """
        return (self.buffer_samples / 2 - self.pretrigger_samples) / self.sample_rate_hz


@dataclasses.dataclass
class Capture:
    sample_rate_hz: float
    trigger_index: int  # index of the trigger sample in ch1_v / ch2_v
    auto_triggered: bool  # True: auto timeout fired, not a real trigger
    ch1_v: np.ndarray
    ch2_v: np.ndarray  # empty when ch2 is disabled
    device_time: tuple  # (secUtc, tick, ticksPerSecond) from FDwfAnalogInStatusTime
    done_time: object = None  # value of poll(now=...) taken when Done was seen, before read-out


def _int(v=0):
    return ctypes.c_int(v)


class AnalogDiscoveryScope:
    """The scope (analog in) of one Analog Discovery."""

    def __init__(self, dwf=None, serial='', config_index=-1):
        self._dwf = dwf
        self.serial = serial  # '' = first free AD3/AD2
        self.config_index = config_index  # -1 = the configuration with the largest scope buffer
        self._hdwf = ctypes.c_int(d.HDWF_NONE)
        self.settings = None
        self.device_name = None
        self.max_buffer_samples = None
        self.record_settings = None

    @property
    def dwf(self):
        if self._dwf is None:
            self._dwf = d.Dwf()
        return self._dwf

    # --- device selection ---

    def list_devices(self):
        """Every attached AD3/AD2 as dicts, without opening anything."""
        dwf = self.dwf
        count = _int()
        dwf.call('FDwfEnum', _int(d.ENUMFILTER_ALL), ctypes.pointer(count))
        devices = []
        for i in range(count.value):
            devid, devver, used = _int(), _int(), _int()
            dwf.call('FDwfEnumDeviceType', _int(i), ctypes.pointer(devid), ctypes.pointer(devver))
            if devid.value not in d.DEVICE_NAMES:
                continue
            dwf.call('FDwfEnumDeviceIsOpened', _int(i), ctypes.pointer(used))
            sn = ctypes.create_string_buffer(32)
            dwf.call('FDwfEnumSN', _int(i), sn)
            # The SDK reports 'SN:210415BF81F3'; keep the bare serial that lsusb shows.
            serial = sn.value.decode().removeprefix('SN:')
            devices.append({'index': i, 'name': d.DEVICE_NAMES[devid.value],
                            'serial': serial, 'in_use': bool(used.value)})
        return devices

    def _config_buffers(self, index):
        """Scope buffer size (samples) of each device configuration."""
        dwf = self.dwf
        count = _int()
        dwf.call('FDwfEnumConfig', _int(index), ctypes.pointer(count))
        sizes = []
        for cfg in range(count.value):
            size = _int()
            dwf.call('FDwfEnumConfigInfo', _int(cfg), _int(d.DECI_ANALOG_IN_BUFFER_SIZE),
                     ctypes.pointer(size))
            sizes.append(size.value)
        return sizes

    def open(self):
        devices = self.list_devices()
        if self.serial:
            matches = [dev for dev in devices if dev['serial'] == self.serial]
        else:
            matches = [dev for dev in devices if not dev['in_use']]
        if not matches:
            raise d.DwfError(f'no free Analog Discovery with serial {self.serial or "(any)"}; '
                             f'attached: {devices}')
        dev = matches[0]
        if dev['in_use']:
            raise d.DwfError(f'{dev["name"]} {dev["serial"]} is opened by another process')
        sizes = self._config_buffers(dev['index'])
        cfg = self.config_index
        if cfg < 0:
            cfg = int(np.argmax(sizes))
        if not 0 <= cfg < len(sizes):
            raise ValueError(f'config_index {cfg} not in 0..{len(sizes) - 1}')
        self.dwf.call('FDwfDeviceConfigOpen', _int(dev['index']), _int(cfg),
                      ctypes.pointer(self._hdwf))
        self.device_name = f'{dev["name"]} {dev["serial"]} (config {cfg})'
        self.serial = dev['serial']
        self.max_buffer_samples = sizes[cfg]
        # The open succeeds even when the analog circuit stays off (seen with an
        # AD3 behind two USB 2.0 hubs: it needs 600 mA); acquisitions then never
        # start. The SDK only reports this in the last error message.
        message = self.dwf.last_error()
        if 'turned off' in message:
            self.close()
            raise d.DwfError(f'{self.device_name}: {message}')

    def close(self):
        if self._hdwf.value != d.HDWF_NONE:
            self.dwf.lib.FDwfDeviceClose(self._hdwf)  # no error check: closing must not raise
            self._hdwf = ctypes.c_int(d.HDWF_NONE)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # --- continuous recording (see RecordSettings) ---

    def configure_record(self, settings, settle_s=DEFAULT_SETTLE_S):
        """Set up a continuous CH1 recording (acqmodeRecord); return the device's rate (Hz).

        The number of samples (duration_s x sample_rate_hz asked for) is kept; when the
        device rounds the rate, record_settings.duration_s follows it.
        """
        return _configure_record(self, settings, settle_s)

    def record(self, should_stop=None, now=None, poll_period_s=0.001,
               stall_timeout_s=None):
        """Start the configured recording and read it to the end (blocking); return a Recording.

        now() (if given) is called right after the start command, for the start stamp.
        should_stop() is checked between reads; RecordingAborted is raised when it is true.
        Can be called again for the next recording with the same settings.
        """
        return _record(self, should_stop, now, poll_period_s,
                       RECORD_STALL_TIMEOUT_S if stall_timeout_s is None else stall_timeout_s)

    # --- acquisition ---

    def configure(self, settings, settle_s=DEFAULT_SETTLE_S):
        """Apply settings, wait settle_s, return the sample rate the device uses (Hz)."""
        if not (math.isfinite(settle_s) and settle_s >= 0.0):
            raise ValueError('settle_s must be a finite, non-negative number')
        settings.validate()
        if settings.buffer_samples > self.max_buffer_samples:
            raise ValueError(f'buffer_samples {settings.buffer_samples} exceeds the '
                             f'{self.max_buffer_samples} samples of this configuration')
        h, call = self._hdwf, self.dwf.call
        call('FDwfAnalogInReset', h)
        for ch, enabled, range_peak_v in ((0, True, settings.ch1_range_peak_v),
                                          (1, settings.ch2_enabled, settings.ch2_range_peak_v)):
            call('FDwfAnalogInChannelEnableSet', h, _int(ch), _int(int(enabled)))
            if not enabled:
                continue
            call('FDwfAnalogInChannelRangeSet', h, _int(ch), ctypes.c_double(2 * range_peak_v))
            call('FDwfAnalogInChannelOffsetSet', h, _int(ch), ctypes.c_double(0.0))
            actual = ctypes.c_double()
            call('FDwfAnalogInChannelRangeGet', h, _int(ch), ctypes.pointer(actual))
            if actual.value / 2 < range_peak_v * 0.999:
                raise ValueError(f'ch{ch + 1}: device range +-{actual.value / 2:g} V is '
                                 f'narrower than the requested +-{range_peak_v:g} V')
        call('FDwfAnalogInFrequencySet', h, ctypes.c_double(settings.sample_rate_hz))
        call('FDwfAnalogInBufferSizeSet', h, _int(settings.buffer_samples))
        call('FDwfAnalogInAcquisitionModeSet', h, _int(d.ACQMODE_SINGLE))
        if settings.trigger_source == 'none':
            call('FDwfAnalogInTriggerSourceSet', h, ctypes.c_ubyte(d.TRIGSRC_NONE))
        else:
            call('FDwfAnalogInTriggerSourceSet', h, ctypes.c_ubyte(d.TRIGSRC_DETECTOR_ANALOG_IN))
            call('FDwfAnalogInTriggerTypeSet', h, _int(d.TRIGTYPE_EDGE))
            call('FDwfAnalogInTriggerChannelSet', h,
                 _int(0 if settings.trigger_source == 'ch1' else 1))
            call('FDwfAnalogInTriggerConditionSet', h,
                 _int(d.TRIGGER_SLOPES[settings.trigger_slope]))
            call('FDwfAnalogInTriggerLevelSet', h, ctypes.c_double(settings.trigger_level_v))
            call('FDwfAnalogInTriggerHysteresisSet', h,
                 ctypes.c_double(settings.trigger_hysteresis_v))
        call('FDwfAnalogInTriggerAutoTimeoutSet', h, ctypes.c_double(settings.auto_timeout_s))

        rate = ctypes.c_double()
        call('FDwfAnalogInFrequencyGet', h, ctypes.pointer(rate))
        size = _int()
        call('FDwfAnalogInBufferSizeGet', h, ctypes.pointer(size))
        if size.value != settings.buffer_samples:
            raise ValueError(f'device took buffer_samples {size.value}, '
                             f'not {settings.buffer_samples}')
        # The trigger position depends on the actual rate, so set it last.
        applied = dataclasses.replace(settings, sample_rate_hz=rate.value)
        call('FDwfAnalogInTriggerPositionSet', h, ctypes.c_double(applied.trigger_position_s()))
        self.settings = applied
        time.sleep(settle_s)  # let the input transient after reconfiguring die out
        return rate.value

    def arm(self):
        """Start (or restart) one acquisition."""
        self.dwf.call('FDwfAnalogInConfigure', self._hdwf, _int(1), _int(1))

    def poll(self, now=None):
        """Return a Capture once the acquisition is done, else None.

        Completion is checked without transferring data, and now() (if given)
        is called right then, before the read-out (4.5 ms for 2 x 8192 samples
        on the AD3), so done_time is not delayed by the USB transfer.
        """
        h, call, s = self._hdwf, self.dwf.call, self.settings
        state = ctypes.c_ubyte()
        call('FDwfAnalogInStatus', h, _int(0), ctypes.pointer(state))
        if state.value != d.STATE_DONE:
            return None
        done_time = now() if now is not None else None
        call('FDwfAnalogInStatus', h, _int(1), ctypes.pointer(state))  # transfers the data
        data = []
        for ch, enabled in ((0, True), (1, s.ch2_enabled)):
            if not enabled:
                data.append(np.zeros(0))
                continue
            buf = (ctypes.c_double * s.buffer_samples)()
            call('FDwfAnalogInStatusData', h, _int(ch), buf, _int(s.buffer_samples))
            data.append(np.frombuffer(buf, dtype=np.float64).copy())
        auto = _int()
        call('FDwfAnalogInStatusAutoTriggered', h, ctypes.pointer(auto))
        sec, tick, tps = ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint()
        call('FDwfAnalogInStatusTime', h, ctypes.pointer(sec), ctypes.pointer(tick),
             ctypes.pointer(tps))
        position = ctypes.c_double()
        call('FDwfAnalogInTriggerPositionStatus', h, ctypes.pointer(position))
        # Same convention as CaptureSettings.trigger_position_s (UNVERIFIED sign).
        index = round(s.buffer_samples / 2 - position.value * s.sample_rate_hz)
        return Capture(sample_rate_hz=s.sample_rate_hz,
                       trigger_index=min(max(index, 0), s.buffer_samples - 1),
                       auto_triggered=bool(auto.value), ch1_v=data[0], ch2_v=data[1],
                       device_time=(sec.value, tick.value, tps.value), done_time=done_time)


# --- continuous recording (acqmodeRecord) ---

# Recording fails when the device delivers no sample for this long after it started.
RECORD_STALL_TIMEOUT_S = 2.0


@dataclasses.dataclass
class RecordSettings:
    """One continuous recording of CH1 (acqmodeRecord), started without a trigger.

    Unlike a triggered capture, the samples are streamed to the host while the
    device records, so the length is limited by host memory, not the device buffer.
    Samples the host did not read in time are lost (counted, the gap stays 0 V).
    """

    sample_rate_hz: float = 2e6  # ROS 1 AE recording: 5e-7 s per sample
    duration_s: float = 12.0
    ch1_range_peak_v: float = 2.5  # +- V; AD3 has +-2.5 V and +-25 V

    def validate(self):
        for name in ('sample_rate_hz', 'duration_s', 'ch1_range_peak_v'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be a positive number')
        if self.sample_rate_hz > MAX_SAMPLE_RATE_HZ:
            raise ValueError(f'sample_rate_hz must be at most {MAX_SAMPLE_RATE_HZ:g}')
        if self.num_samples() < 2:
            raise ValueError('duration_s x sample_rate_hz must give at least 2 samples')

    def num_samples(self):
        return round(self.duration_s * self.sample_rate_hz)


@dataclasses.dataclass
class Recording:
    sample_rate_hz: float
    ch1_v: np.ndarray  # float32, V; num_samples long
    lost_samples: int  # the device overwrote them before the host read them (left at 0 V)
    corrupted_samples: int  # the SDK flagged them as possibly overwritten while read
    start_time: object = None  # value of record(now=...) taken right after the start


class RecordingAborted(RuntimeError):
    """record() was stopped through should_stop."""


def _configure_record(scope, settings, settle_s):
    if not (math.isfinite(settle_s) and settle_s >= 0.0):
        raise ValueError('settle_s must be a finite, non-negative number')
    settings.validate()
    h, call = scope._hdwf, scope.dwf.call
    call('FDwfAnalogInReset', h)
    call('FDwfAnalogInChannelEnableSet', h, _int(0), _int(1))
    call('FDwfAnalogInChannelEnableSet', h, _int(1), _int(0))
    call('FDwfAnalogInChannelRangeSet', h, _int(0), ctypes.c_double(2 * settings.ch1_range_peak_v))
    call('FDwfAnalogInChannelOffsetSet', h, _int(0), ctypes.c_double(0.0))
    actual = ctypes.c_double()
    call('FDwfAnalogInChannelRangeGet', h, _int(0), ctypes.pointer(actual))
    if actual.value / 2 < settings.ch1_range_peak_v * 0.999:
        raise ValueError(f'ch1: device range +-{actual.value / 2:g} V is narrower than '
                         f'the requested +-{settings.ch1_range_peak_v:g} V')
    call('FDwfAnalogInAcquisitionModeSet', h, _int(d.ACQMODE_RECORD))
    call('FDwfAnalogInFrequencySet', h, ctypes.c_double(settings.sample_rate_hz))
    rate = ctypes.c_double()
    call('FDwfAnalogInFrequencyGet', h, ctypes.pointer(rate))
    # keep the number of samples asked for; the duration follows the device's rate
    applied = dataclasses.replace(settings, sample_rate_hz=rate.value,
                                  duration_s=settings.num_samples() / rate.value)
    call('FDwfAnalogInRecordLengthSet', h, ctypes.c_double(applied.duration_s))
    call('FDwfAnalogInTriggerSourceSet', h, ctypes.c_ubyte(d.TRIGSRC_NONE))
    call('FDwfAnalogInConfigure', h, _int(1), _int(0))  # apply without starting
    scope.record_settings = applied
    time.sleep(settle_s)  # let the input settle after reconfiguring
    return rate.value


def _record(scope, should_stop=None, now=None, poll_period_s=0.001,
            stall_timeout_s=RECORD_STALL_TIMEOUT_S, clock=time.monotonic):
    s = scope.record_settings
    if s is None:
        raise RuntimeError('configure_record() first')
    h, call = scope._hdwf, scope.dwf.call
    n = s.num_samples()
    out = np.zeros(n, dtype=np.float32)
    chunk = np.empty(0, dtype=np.float64)
    state, available, lost, corrupted = ctypes.c_ubyte(), _int(), _int(), _int()
    got = lost_total = corrupted_total = 0
    call('FDwfAnalogInConfigure', h, _int(0), _int(1))  # start
    start_time = now() if now is not None else None
    last_progress = clock()
    try:
        while got < n:
            if should_stop is not None and should_stop():
                raise RecordingAborted('recording stopped')
            call('FDwfAnalogInStatus', h, _int(1), ctypes.pointer(state))
            if got == 0 and state.value in (d.STATE_CONFIG, d.STATE_PREFILL, d.STATE_ARMED):
                pass  # not recording yet
            else:
                call('FDwfAnalogInStatusRecord', h, ctypes.pointer(available),
                     ctypes.pointer(lost), ctypes.pointer(corrupted))
                lost_total += lost.value
                corrupted_total += corrupted.value
                got = min(n, got + lost.value)  # the lost samples come first; they stay 0 V
                take = min(available.value, n - got)
                if take > 0:
                    if len(chunk) < take:
                        chunk = np.empty(take, dtype=np.float64)
                    call('FDwfAnalogInStatusData', h, _int(0),
                         chunk.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), _int(take))
                    out[got:got + take] = chunk[:take]
                    got += take
                if take > 0 or lost.value > 0:
                    last_progress = clock()
                if got < n and state.value == d.STATE_DONE and available.value == 0:
                    raise d.DwfError(f'recording ended after {got} of {n} samples')
            if got < n:
                if clock() - last_progress > stall_timeout_s:
                    raise d.DwfError(f'no samples from the device for {stall_timeout_s} s '
                                     f'({got} of {n} recorded)')
                time.sleep(poll_period_s)
    finally:
        scope.dwf.lib.FDwfAnalogInConfigure(h, _int(0), _int(0))  # stop; must not raise
    return Recording(sample_rate_hz=s.sample_rate_hz, ch1_v=out, lost_samples=lost_total,
                     corrupted_samples=corrupted_total, start_time=start_time)
