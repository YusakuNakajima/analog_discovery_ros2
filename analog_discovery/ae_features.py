"""Acoustic-emission hit features from one triggered capture.

Hit-based parameters in the usual AE sense (ASTM E1316 terms), computed
against a fixed detection threshold on the signal minus its pre-trigger mean:

    peak_amplitude_v  largest |v| in the capture after the trigger (V)
    rise_time_s       first threshold crossing -> peak
    duration_s        first -> last threshold crossing
    counts            upward crossings of +threshold
    energy_v2s        integral of v^2 over the duration (V^2 s)
    rms_v             RMS over the duration
    peak_frequency_hz, centroid_frequency_hz   spectrum over the duration

The hit is "truncated" when the signal is still above the threshold near the
end of the capture, i.e. duration and energy are lower bounds.

Amplitudes are at the AE9501B output (its gain is not removed), so they are
comparable between runs only with the same gain setting.
"""

import dataclasses

import numpy as np

# A last threshold crossing within this time of the end of the capture means the
# hit outlasted it. A ringing signal dips below the threshold every half period,
# so the margin must cover the half period of the lowest frequency that matters:
# 50 us covers down to 10 kHz (the real AE hits peaked at 45-95 kHz). With 1.6 us
# a hit ringing to the very end of a capture was reported as complete.
TRUNCATION_MARGIN_S = 50e-6


@dataclasses.dataclass
class HitFeatures:
    peak_amplitude_v: float
    rise_time_s: float
    duration_s: float
    counts: int
    energy_v2s: float
    rms_v: float
    peak_frequency_hz: float
    centroid_frequency_hz: float
    noise_rms_v: float
    truncated: bool


def hit_features(samples_v, sample_rate_hz, trigger_index, threshold_v):
    """Features of the hit in one capture, or None if nothing exceeds threshold_v."""
    if not threshold_v > 0.0:
        raise ValueError('threshold_v must be positive')
    v = np.asarray(samples_v, dtype=np.float64)
    pre = v[:trigger_index]
    baseline_v = float(pre.mean()) if len(pre) else 0.0
    noise_rms_v = float(pre.std()) if len(pre) else 0.0
    x = v - baseline_v
    # Look from a little before the trigger: the crossing that fired it can sit
    # a sample or two early in the record.
    start = max(trigger_index - 8, 0)
    above = np.nonzero(np.abs(x[start:]) >= threshold_v)[0]
    if len(above) == 0:
        return None
    first = start + int(above[0])
    last = start + int(above[-1])
    peak = first + int(np.argmax(np.abs(x[first:last + 1])))
    segment = x[first:last + 1]
    dt_s = 1.0 / sample_rate_hz
    rising = (x[first:last] < threshold_v) & (x[first + 1:last + 1] >= threshold_v)
    counts = int(np.count_nonzero(rising)) + int(x[first] >= threshold_v)
    spectrum = np.abs(np.fft.rfft(segment * np.hanning(len(segment)))) if len(segment) > 1 \
        else np.zeros(1)
    freqs_hz = np.fft.rfftfreq(len(segment), dt_s) if len(segment) > 1 else np.zeros(1)
    power = spectrum ** 2
    centroid_hz = float((freqs_hz * power).sum() / power.sum()) if power.sum() > 0 else 0.0
    return HitFeatures(
        peak_amplitude_v=float(abs(x[peak])),
        rise_time_s=(peak - first) * dt_s,
        duration_s=(last - first) * dt_s,
        counts=counts,
        energy_v2s=float((segment ** 2).sum() * dt_s),
        rms_v=float(np.sqrt((segment ** 2).mean())),
        peak_frequency_hz=float(freqs_hz[np.argmax(spectrum[1:]) + 1]) if len(spectrum) > 1
        else 0.0,
        centroid_frequency_hz=centroid_hz,
        noise_rms_v=noise_rms_v,
        truncated=(len(v) - 1 - last) * dt_s <= TRUNCATION_MARGIN_S,
    )


class ActivityWindow:
    """Hit rate and energy rate over a sliding time window (seconds, any clock)."""

    def __init__(self, window_s):
        if not window_s > 0.0:
            raise ValueError('window_s must be positive')
        self.window_s = window_s
        self._hits = []  # (t_s, HitFeatures)

    def add(self, t_s, features):
        self._hits.append((t_s, features))

    def summary(self, now_s):
        """Dict of window statistics at now_s; hits older than the window are dropped."""
        self._hits = [(t, f) for t, f in self._hits if now_s - t <= self.window_s]
        hits = [f for _, f in self._hits]
        n = len(hits)
        return {
            'window_s': self.window_s,
            'hits': n,
            'hit_rate_hz': n / self.window_s,
            'energy_rate_v2': sum(f.energy_v2s for f in hits) / self.window_s,
            'mean_peak_amplitude_v': float(np.mean([f.peak_amplitude_v for f in hits])) if n
            else 0.0,
            'max_peak_amplitude_v': max((f.peak_amplitude_v for f in hits), default=0.0),
            'counts_rate_hz': sum(f.counts for f in hits) / self.window_s,
            'truncated_hits': sum(f.truncated for f in hits),
        }
