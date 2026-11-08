"""Continuous AE recordings, one per grinding cycle, as in ROS 1 grinding_takamoto_with_AE.

ROS 1 (onolab_ros1 grinding_vibration acoustic_emission_service.py + analysis_ae.py):
"start" made a Windows PC record the AE signal at 2 MHz (5e-7 s per sample) into a CSV
of ADC codes on the NAS; "get" read the newest CSV, skipped the first 200012 samples
(start_sample_index 200013, 1-based; 0.1 s) and returned

    ae_integral = sum over FFT bins with f > 100 kHz of (|X(f)| * 2 / N)^2     [V^2]

Here the AD3 records CH1 (the AE9501B RF output) itself in Record mode. AeRecorder
runs one recording at a time in a thread: start() returns once the device records,
the recording is then saved (.npy of float32 V + .json metadata) and its ae_integral
appended to a CSV, whether or not get() is called; get() waits for that and returns it.
A saved recording is NOT valid until the caller confirm()s it (its cycle went as
planned): any failure, interruption or lost answer on the caller's side leaves it
unconfirmed, without the caller having to clean up.

The value is NOT comparable with the ROS 1 targets: another ADC (ROS 1 mapped codes
to +-1 V; the AD3 measures V at its input) and possibly another AE9501B gain.
"""

import csv
import dataclasses
import datetime
import json
import os
import shutil
import threading

import numpy as np

from analog_discovery.scope import RecordingAborted

FEATURE_NAME = 'ae_spectrum_integral'
FEATURE_UNIT = 'V^2'
ROS1_SKIP_SAMPLES = 200012  # ROS 1 start_sample_index 200013 (1-based) = 0.1 s at 2 MHz
ROS1_FREQUENCY_THRESHOLD_HZ = 100e3
FREE_SPACE_MARGIN_BYTES = 200 * 1024 * 1024  # left free on the disk after a recording
CSV_FIELDS = ('index', 'start_wall_time', 'start_ros_time_s', 'file', 'sample_rate_hz',
              'num_samples', 'lost_samples', 'corrupted_samples', 'skip_samples',
              'frequency_threshold_hz', 'ae_spectrum_integral_v2', 'complete', 'confirmed', 'valid',
              'note')
# A recording gets a row when it is saved (confirmed False, so valid False) and a second row
# with the same index when the caller confirms it (valid True); the last row of an index
# wins. The .json is authoritative ("valid"), the CSV is a log of it.


def spectrum_integral_v2(voltage_v, sample_rate_hz, skip_samples=ROS1_SKIP_SAMPLES,
                         frequency_threshold_hz=ROS1_FREQUENCY_THRESHOLD_HZ):
    """ROS 1 Analysis_AE_Data.spectrum_integration on voltage_v (V) -> V^2.

    ROS 1: X = fft(v[skip:]), f = fftfreq(N, 1/fs), bins with f > threshold,
    amplitude |X| * 2 / N, sum of squares. rfft gives the same positive bins; for an
    even N its last bin (+fs/2) is the one fftfreq lists as -fs/2, which ROS 1 left out.
    """
    v = np.asarray(voltage_v, dtype=np.float64)[skip_samples:]
    n = len(v)
    if n < 2:
        raise ValueError(f'{n} samples after skipping {skip_samples}: recording too short')
    spectrum = np.fft.rfft(v)
    frequency_hz = np.fft.rfftfreq(n, 1.0 / sample_rate_hz)
    mask = frequency_hz > frequency_threshold_hz
    if n % 2 == 0:
        mask[-1] = False
    amplitude_v = np.abs(spectrum[mask]) * 2.0 / n
    return float(np.sum(amplitude_v ** 2))


@dataclasses.dataclass
class RecordResult:
    index: int
    file: str  # .npy path ('' if saving failed)
    sample_rate_hz: float = 0.0
    num_samples: int = 0
    lost_samples: int = 0
    corrupted_samples: int = 0
    value_v2: float = float('nan')
    complete: bool = False  # saved, no lost / corrupted samples, value computed
    confirmed: bool = False  # the caller confirmed it matches what it measured (confirm())
    error: str = ''
    start_wall_time: str = ''
    start_ros_time_s: float = float('nan')

    def message(self):
        text = (f'{FEATURE_NAME}={self.value_v2:.9g} {FEATURE_UNIT} index={self.index} '
                f'file={self.file} samples={self.num_samples} lost={self.lost_samples} '
                f'corrupted={self.corrupted_samples}')
        return text if not self.error else f'{self.error}; {text}'


class AeRecorder:
    """One recording at a time on a configured scope (configure_record() done)."""

    def __init__(self, scope, record_dir, feature_log_csv='', skip_samples=ROS1_SKIP_SAMPLES,
                 frequency_threshold_hz=ROS1_FREQUENCY_THRESHOLD_HZ, ros_time_s=None,
                 log=None, on_result=None, extra_metadata=None):
        if not record_dir:
            raise ValueError('record_dir must be set: the raw AE recordings are the point')
        os.makedirs(record_dir, exist_ok=True)
        self.scope = scope
        self.record_dir = record_dir
        self.feature_log_csv = feature_log_csv
        self.skip_samples = int(skip_samples)
        self.frequency_threshold_hz = float(frequency_threshold_hz)
        if self.skip_samples < 0:
            raise ValueError('skip_samples must not be negative')
        nyquist_hz = scope.record_settings.sample_rate_hz / 2
        if not 0.0 <= self.frequency_threshold_hz < nyquist_hz:  # NaN fails too
            raise ValueError(f'frequency_threshold_hz {self.frequency_threshold_hz} must be in '
                             f'[0, {nyquist_hz:g}) (half the sample rate): no FFT bin above it '
                             'would make every value 0')
        if self.skip_samples + 2 > scope.record_settings.num_samples():
            raise ValueError(f'skip_samples {self.skip_samples} leaves less than 2 of the '
                             f'{scope.record_settings.num_samples()} recorded samples')
        self.ros_time_s = ros_time_s or (lambda: float('nan'))
        self.log = log or (lambda text: None)
        self.on_result = on_result or (lambda result: None)  # called in the recording thread
        self.extra_metadata = dict(extra_metadata or {})
        self._lock = threading.Lock()
        self._confirm_lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._index = 0
        self._result = None  # RecordResult of the last finished recording, until get()
        self._last_read = None  # RecordResult last returned by get, until confirmed
        self._phase = 'idle'  # recording | saving (while the thread runs)

    # --- commands (thread-safe) ---

    def start(self, timeout_s=2.0):
        """Start a recording; return (ok, message) once samples arrive from the device."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False, f'recording {self._index} is still {self._phase}'
            if self._result is not None:
                self.log(f'recording {self._result.index} was not read with get '
                         f'(saved: {self._result.file or "no"})')
                self._result = None
            needed = self.scope.record_settings.num_samples() * 4 + FREE_SPACE_MARGIN_BYTES
            free = shutil.disk_usage(self.record_dir).free
            if free < needed:
                return False, (f'{free / 1e9:.2f} GB free in {self.record_dir}, a recording '
                               f'needs {needed / 1e9:.2f} GB')
            self._index += 1
            index = self._index
            self._stop.clear()
            self._phase = 'recording'
            started = threading.Event()
            info = {}
            self._thread = threading.Thread(target=self._run, args=(index, started, info),
                                            daemon=True)
            self._thread.start()
        if not started.wait(timeout_s):
            self._stop.set()  # a recording that starts later is aborted, not saved
            return False, f'no samples from the device within {timeout_s} s'
        if 'error' in info:
            return False, info['error']
        return True, f'recording {index} started ({self._describe()})'

    def get(self, timeout_s):
        """Wait until the last started recording is saved; return (ok, RecordResult or message).

        Bound to that recording: if another start replaced it meanwhile, nothing is returned.
        """
        with self._lock:
            thread, index = self._thread, self._index
        if thread is None:
            return False, 'no recording (start first)'
        thread.join(timeout_s)
        if thread.is_alive():
            return False, f'recording {index} not finished within {timeout_s} s'
        with self._lock:
            result = self._result
            if result is None or result.index != index:
                return False, (f'no unread result of recording {index} '
                               '(already read, aborted, or replaced by a newer start)')
            self._result = None
            self._last_read = result
        return result.complete, result

    def stop(self, timeout_s=5.0):
        """Abort a recording still being recorded (nothing is saved). Returns (ok, message).

        A recording already being saved is not aborted: stop waits for it and says so.
        Only the result of that recording is dropped, never one of a later start.
        """
        with self._lock:
            thread, index, phase = self._thread, self._index, self._phase
            if thread is None or not thread.is_alive():
                dropped = self._take_result(index)
                return True, ('nothing running' if dropped is None
                              else f'result of recording {index} dropped (files kept)')
            if phase == 'recording':
                self._stop.set()
        thread.join(timeout_s)
        if thread.is_alive():
            return False, f'recording {index} did not finish within {timeout_s} s'
        with self._lock:
            dropped = self._take_result(index)
        if dropped is None:
            return True, f'recording {index} aborted (nothing saved)'
        return True, (f'recording {index} had already finished and is saved '
                      f'({dropped.file or "not saved"}); its result is dropped')

    def confirm(self):
        """Confirm the recording last returned by get: the caller's cycle went as planned.

        Only a complete recording can be confirmed. Writes confirmed / valid true into its
        .json and a second CSV row (valid True). Retryable when the write fails; a later
        start does not change which recording this confirms. Returns (ok, message).
        """
        with self._confirm_lock:  # one confirmation at a time, answered after it is written
            with self._lock:
                result = self._last_read
            if result is None:
                return False, 'no recording returned by get to confirm'
            if result.confirmed:
                return True, f'recording {result.index} already confirmed'
            if not result.complete:
                return False, f'recording {result.index} is not complete ({result.error})'
            confirmed = dataclasses.replace(result, confirmed=True)
            # The .json decides; it is written last. The CSV is a log (its last row of an
            # index wins): a failed .json write is followed by a valid False row.
            try:
                self._log_row(confirmed, note='confirmed')
            except Exception as e:
                return False, f'recording {result.index}: confirmation not written ({e}); try again'
            try:
                self._write_meta(result.file, {'confirmed': True, 'valid': True})
            except Exception as e:
                try:
                    self._log_row(result, note=f'confirm failed: {e}')
                except Exception:
                    pass
                return False, f'recording {result.index}: confirmation not written ({e}); try again'
            result.confirmed = True  # only once both are written
            self.on_result(result)
            return True, f'recording {result.index} confirmed'

    def _take_result(self, index):
        """Under the lock: drop and return the unread result if it is recording index's."""
        if self._result is not None and self._result.index == index:
            result, self._result = self._result, None
            return result
        return None

    def busy(self):
        return self._thread is not None and self._thread.is_alive()

    # --- worker ---

    def _describe(self):
        s = self.scope.record_settings
        return f'{s.num_samples()} samples at {s.sample_rate_hz:g} Hz = {s.duration_s:g} s'

    def _run(self, index, started, info):
        wall = {}

        def on_start():
            wall['time'] = datetime.datetime.now().astimezone()
            wall['ros_time_s'] = self.ros_time_s()
            started.set()

        try:
            rec = self.scope.record(should_stop=self._stop.is_set, now=on_start)
        except RecordingAborted:
            self.log(f'recording {index} aborted (nothing saved)')
            info['error'] = f'recording {index} aborted'
            started.set()
            return
        except Exception as e:  # SDK error: report it through get
            info['error'] = f'recording {index} failed: {e}'
            started.set()
            self._finish(RecordResult(index=index, file='', error=info['error']))
            return
        with self._lock:
            if self._stop.is_set():  # stop came after the last read, before the save
                info['error'] = f'recording {index} aborted'
                started.set()
                self.log(f'recording {index} aborted (nothing saved)')
                return
            self._phase = 'saving'  # from here stop no longer aborts
        result = RecordResult(index=index, file='', sample_rate_hz=rec.sample_rate_hz,
                              num_samples=len(rec.ch1_v), lost_samples=rec.lost_samples,
                              corrupted_samples=rec.corrupted_samples)
        errors = []
        try:
            result.value_v2 = spectrum_integral_v2(rec.ch1_v, rec.sample_rate_hz,
                                                   self.skip_samples, self.frequency_threshold_hz)
        except Exception as e:
            errors.append(f'{FEATURE_NAME} not computed: {e}')
        if rec.lost_samples or rec.corrupted_samples:
            errors.append(f'{rec.lost_samples} samples lost, {rec.corrupted_samples} corrupted '
                          '(the host did not read the device fast enough); value not valid')
        result.start_wall_time = wall['time'].isoformat()
        result.start_ros_time_s = wall['ros_time_s']
        self._save(index, wall, rec, result, errors)
        result.error = '; '.join(errors)
        result.complete = not errors
        self._finish(result)

    def _finish(self, result):
        with self._lock:
            self._result = result
        self.log(result.message())
        self.on_result(result)

    def _save(self, index, wall, rec, result, errors):
        """Write .npy, .json, the CSV row; failures go to errors, and the .json and the CSV
        row always end with the final complete flag (a failed CSV row is noted in the .json).
        Saved recordings are not valid until confirm()."""
        stamp = wall['time'].strftime('%Y%m%d_%H%M%S')
        base = os.path.join(self.record_dir, f'ae_{stamp}_{index:04d}')
        npy = base + '.npy'
        try:
            with open(npy, 'xb') as f:  # never overwrite an earlier recording
                np.save(f, rec.ch1_v.astype(np.float32, copy=False))
        except Exception as e:
            errors.append(f'not saved: {e}')
            return
        result.file = npy
        s = self.scope.record_settings
        meta = {
            'file': os.path.basename(npy),
            'index': index,
            'start_wall_time': result.start_wall_time,
            'start_ros_time_s': result.start_ros_time_s,
            'channel': 'ch1',
            'unit': 'V',
            'dtype': 'float32',
            'sample_rate_hz': rec.sample_rate_hz,
            'num_samples': len(rec.ch1_v),
            'duration_s': len(rec.ch1_v) / rec.sample_rate_hz,
            'ch1_range_peak_v': s.ch1_range_peak_v,
            'device': getattr(self.scope, 'device_name', None),
            'lost_samples': rec.lost_samples,
            'corrupted_samples': rec.corrupted_samples,
            'feature': {
                'name': FEATURE_NAME, 'unit': FEATURE_UNIT, 'value': result.value_v2,
                'skip_samples': self.skip_samples,
                'frequency_threshold_hz': self.frequency_threshold_hz,
                'definition': 'ROS 1 analysis_ae.py spectrum_integration',
            },
            'errors': errors,
            'complete': not errors,
            'confirmed': False,  # set by confirm(): the caller's cycle went as planned
            'valid': False,  # = complete and confirmed; use only valid recordings
            **self.extra_metadata,
        }
        try:
            with open(base + '.json', 'x') as f:
                json.dump(meta, f, indent=1)
        except Exception as e:
            errors.append(f'metadata not saved: {e}')
        try:
            result.complete = not errors
            self._log_row(result, note='saved')
        except Exception as e:
            errors.append(f'CSV row not written: {e}')
            try:
                self._write_meta(npy, {'errors': errors, 'complete': False})
            except Exception:
                pass  # already reported through errors (get fails)

    def _write_meta(self, npy, changes):
        meta_path = npy[:-4] + '.json'
        with open(meta_path) as f:
            meta = json.load(f)
        meta.update(changes)
        tmp = meta_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(meta, f, indent=1)
        os.replace(tmp, meta_path)

    def _log_row(self, result, note=''):
        if not self.feature_log_csv:
            return
        log = self.feature_log_csv
        new = not os.path.exists(log) or os.path.getsize(log) == 0
        with open(log, 'a', newline='') as f:
            writer = csv.writer(f)
            if new:
                writer.writerow(CSV_FIELDS)
            writer.writerow([result.index, result.start_wall_time, result.start_ros_time_s, result.file,
                             result.sample_rate_hz, result.num_samples, result.lost_samples,
                             result.corrupted_samples, self.skip_samples,
                             self.frequency_threshold_hz, result.value_v2, result.complete,
                             result.confirmed, result.complete and result.confirmed, note])
