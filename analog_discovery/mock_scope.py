"""Simulated scope with the AnalogDiscoveryScope API, for testing without a device.

Each capture holds a decaying impulse (exponential, tau = 20 samples) whose
onset is exactly at the trigger index, on ch1 (and half of it on ch2), plus a
little noise. A capture becomes ready period_s after arm().
"""

import time

import numpy as np

from analog_discovery.scope import Capture

IMPULSE_TAU_SAMPLES = 20


class MockScope:

    def __init__(self, serial='', config_index=-1, period_s=0.1, amplitude_v=1.0,
                 noise_v=0.001, clock=time.monotonic, seed=0):
        self.serial = serial or 'MOCK'
        self.device_name = f'mock scope {self.serial}'
        self.max_buffer_samples = 32768  # AD3 configuration 2
        self.settings = None
        self._period_s = period_s
        self._amplitude_v = amplitude_v
        self._noise_v = noise_v
        self._clock = clock
        self._rng = np.random.default_rng(seed)
        self._armed_at = None
        self._is_open = False

    def open(self):
        self._is_open = True

    def close(self):
        self._is_open = False
        self._armed_at = None

    def configure(self, settings):
        settings.validate()
        if settings.buffer_samples > self.max_buffer_samples:
            raise ValueError(f'buffer_samples exceeds {self.max_buffer_samples}')
        self.settings = settings
        return settings.sample_rate_hz

    def arm(self):
        self._armed_at = self._clock()

    def poll(self):
        if self._armed_at is None or self._clock() - self._armed_at < self._period_s:
            return None
        self._armed_at = None
        s = self.settings
        n = np.arange(s.buffer_samples)
        k = n - s.pretrigger_samples
        pulse_v = np.where(k >= 0, self._amplitude_v * np.exp(-k / IMPULSE_TAU_SAMPLES), 0.0)
        if s.trigger_slope == 'falling':
            pulse_v = -pulse_v

        def noisy(v):
            return v + self._rng.normal(0.0, self._noise_v, s.buffer_samples)

        return Capture(sample_rate_hz=s.sample_rate_hz, trigger_index=s.pretrigger_samples,
                       auto_triggered=s.trigger_source == 'none',
                       ch1_v=noisy(pulse_v),
                       ch2_v=noisy(pulse_v / 2) if s.ch2_enabled else np.zeros(0),
                       device_time=(0, 0, 0))
