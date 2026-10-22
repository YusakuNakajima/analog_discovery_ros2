import math

import numpy as np
import pytest
from analog_discovery.ae_features import ActivityWindow, hit_features, HitFeatures

FS_HZ = 10e6
PRE = 4096
N = 32768


def burst(amplitude_v=0.5, freq_hz=100e3, tau_s=200e-6, offset_v=0.0, noise_v=0.0, n=N):
    """Decaying sine starting exactly at the trigger index."""
    t_s = (np.arange(n) - PRE) / FS_HZ
    v = np.where(t_s >= 0, amplitude_v * np.exp(-np.maximum(t_s, 0) / tau_s)
                 * np.sin(2 * math.pi * freq_hz * t_s), 0.0)
    return v + offset_v + np.random.default_rng(1).normal(0.0, noise_v, n)


def test_decaying_sine_features():
    f = hit_features(burst(), FS_HZ, PRE, threshold_v=0.02)
    assert f.peak_amplitude_v == pytest.approx(0.5 * math.exp(-2.5e-6 / 200e-6), rel=0.01)
    assert f.rise_time_s == pytest.approx(2.5e-6, abs=0.2e-6)  # first quarter period
    # above 0.02 V until exp(-t/tau) * 0.5 < 0.02: t = tau * ln(25) = 644 us
    assert f.duration_s == pytest.approx(200e-6 * math.log(25), rel=0.02)
    assert f.counts == pytest.approx(f.duration_s * 100e3, abs=2)
    # integral of (A e^{-t/tau} sin)^2 ~ A^2 tau / 4 (most energy is above the threshold)
    assert f.energy_v2s == pytest.approx(0.25 * 200e-6 / 4, rel=0.02)
    assert f.peak_frequency_hz == pytest.approx(100e3, rel=0.02)
    assert 80e3 < f.centroid_frequency_hz < 150e3
    assert not f.truncated


def test_baseline_offset_and_noise_are_removed():
    clean = hit_features(burst(), FS_HZ, PRE, 0.02)
    shifted = hit_features(burst(offset_v=0.1, noise_v=0.001), FS_HZ, PRE, 0.02)
    assert shifted.peak_amplitude_v == pytest.approx(clean.peak_amplitude_v, rel=0.02)
    assert shifted.noise_rms_v == pytest.approx(0.001, rel=0.1)


def test_a_hit_longer_than_the_capture_is_truncated():
    f = hit_features(burst(tau_s=10e-3), FS_HZ, PRE, 0.02)
    assert f.truncated
    assert f.duration_s == pytest.approx((N - 1 - PRE) / FS_HZ, rel=0.01)


def test_a_slow_ring_reaching_the_end_is_truncated():
    # 20 kHz ring still at full amplitude at the end: its last crossing is up to
    # 25 us before the end, which a sample-count margin missed.
    f = hit_features(burst(freq_hz=20e3, tau_s=1.0), FS_HZ, PRE, 0.02)
    assert f.truncated


def test_a_hit_ending_well_before_the_end_is_not_truncated():
    assert not hit_features(burst(tau_s=500e-6), FS_HZ, PRE, 0.02).truncated


def test_nothing_above_threshold_is_no_hit():
    assert hit_features(burst(amplitude_v=0.01), FS_HZ, PRE, 0.02) is None
    with pytest.raises(ValueError):
        hit_features(burst(), FS_HZ, PRE, 0.0)


def hit(energy, peak=0.5, counts=10, truncated=False):
    return HitFeatures(peak, 1e-6, 1e-4, counts, energy, 0.1, 1e5, 1e5, 1e-3, truncated)


def test_activity_window_rates_and_expiry():
    w = ActivityWindow(1.0)
    w.add(10.0, hit(2e-6, peak=0.4))
    w.add(10.5, hit(4e-6, peak=0.8, truncated=True))
    s = w.summary(10.6)
    assert s['hits'] == 2 and s['hit_rate_hz'] == 2.0
    assert s['energy_rate_v2'] == pytest.approx(6e-6)
    assert s['mean_peak_amplitude_v'] == pytest.approx(0.6)
    assert s['max_peak_amplitude_v'] == 0.8 and s['truncated_hits'] == 1
    s = w.summary(11.2)  # the 10.0 s hit has left the window
    assert s['hits'] == 1 and s['energy_rate_v2'] == pytest.approx(4e-6)
    assert w.summary(20.0)['hits'] == 0
