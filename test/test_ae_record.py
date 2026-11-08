"""Record mode (fake libdwf), the ROS 1 AE value, and AeRecorder (mock scope)."""

import csv
import ctypes
import json
import re
import os
import sys
import threading
import time

import numpy as np
import pytest
from analog_discovery import ae_record
from analog_discovery import dwf as d
from analog_discovery.ae_record import AeRecorder, spectrum_integral_v2
from analog_discovery.mock_scope import MockScope
from analog_discovery.scope import AnalogDiscoveryScope, RecordingAborted, RecordSettings

sys.path.insert(0, os.path.dirname(__file__))
from test_scope import FakeLib  # noqa: E402


# --- the ROS 1 value ---

def ros1_spectrum_integration(voltage_v, time_per_sample_s, skip_samples, threshold_hz):
    """onolab_ros1 analysis_ae.py fft_ae_data + spectrum_integration, without pandas."""
    v = np.asarray(voltage_v, dtype=np.float64)[skip_samples:]
    N = len(v)
    T = time_per_sample_s
    amplitude = np.fft.fft(v)
    frequency = np.fft.fftfreq(N, T)
    mask = frequency > threshold_hz
    amplitude_abs = abs(amplitude[mask]) * 2 / len(frequency)
    return np.sum(amplitude_abs ** 2)


@pytest.mark.parametrize('n', [4000, 4001])  # even N has the -fs/2 bin ROS 1 left out
def test_spectrum_integral_is_the_ros1_value(n):
    rng = np.random.default_rng(1)
    t = np.arange(n) * 5e-7
    v = 0.3 * np.sin(2 * np.pi * 250e3 * t) + 0.1 * np.sin(2 * np.pi * 40e3 * t)
    v += rng.normal(0, 0.01, n)
    v[1::2] += 0.05  # energy at the Nyquist frequency
    ours = spectrum_integral_v2(v, 2e6, skip_samples=123, frequency_threshold_hz=100e3)
    assert ours == pytest.approx(ros1_spectrum_integration(v, 5e-7, 123, 100e3), rel=1e-12)


def test_spectrum_integral_of_a_sine_above_the_threshold():
    n, fs = 20000, 2e6
    v = 0.5 * np.sin(2 * np.pi * 200e3 * np.arange(n) / fs)  # whole number of periods
    # one bin of amplitude 0.5 V
    assert spectrum_integral_v2(v, fs, 0, 100e3) == pytest.approx(0.25, rel=1e-9)
    assert spectrum_integral_v2(v, fs, 0, 300e3) == pytest.approx(0.0, abs=1e-12)


def test_spectrum_integral_needs_samples_after_the_skip():
    with pytest.raises(ValueError):
        spectrum_integral_v2(np.zeros(10), 2e6, skip_samples=9)


# --- Record mode against the fake libdwf ---

class RecordLib(FakeLib):
    """StatusRecord hands out `chunks` = [(available, lost, corrupted), ...], one per status."""

    def __init__(self, chunks, states=None, **kw):
        super().__init__(**kw)
        self.chunks = list(chunks)
        self.states = list(states or [])
        self.next_value = 0.0
        self.record_length_s = None

    def _FDwfAnalogInStatus(self, _h, _read, state):
        state.contents.value = self.states.pop(0) if self.states else d.STATE_TRIGGERED

    def _FDwfAnalogInStatusRecord(self, _h, available, lost, corrupted):
        a, lo, c = self.chunks.pop(0) if self.chunks else (0, 0, 0)
        if lo:
            self.next_value += lo  # the device skipped those
        available.contents.value, lost.contents.value, corrupted.contents.value = a, lo, c

    def _FDwfAnalogInStatusData(self, _h, _ch, buf, n):
        for i in range(n.value):  # sample k has the value k (V)
            buf[i] = self.next_value
            self.next_value += 1.0

    def _FDwfAnalogInRecordLengthSet(self, _h, s):
        self.record_length_s = s.value


def recording_scope(chunks, states=None, n=10, rate=1000.0):
    lib = RecordLib(chunks, states)
    scope = AnalogDiscoveryScope(dwf=d.Dwf(lib))
    scope.open()
    scope.configure_record(RecordSettings(sample_rate_hz=rate, duration_s=n / rate,
                                          ch1_range_peak_v=2.5), settle_s=0.0)
    return scope, lib


def names(lib):
    return [c[0] for c in lib.calls]


def test_configure_record_sets_record_mode_without_trigger_on_ch1_only():
    scope, lib = recording_scope([])
    calls = dict((c[0], c[1]) for c in lib.calls if c[0] in (
        'FDwfAnalogInAcquisitionModeSet', 'FDwfAnalogInTriggerSourceSet',
        'FDwfAnalogInRecordLengthSet'))
    assert calls['FDwfAnalogInAcquisitionModeSet'][1] == d.ACQMODE_RECORD
    assert calls['FDwfAnalogInTriggerSourceSet'][1] == d.TRIGSRC_NONE
    assert lib.record_length_s == pytest.approx(10 / 1000.0)
    enables = [c[1][1:] for c in lib.calls if c[0] == 'FDwfAnalogInChannelEnableSet']
    assert enables == [[0, 1], [1, 0]]
    assert lib.ranges[0] == 5.0  # peak-to-peak
    # applied, not started
    assert [c[1][1:] for c in lib.calls if c[0] == 'FDwfAnalogInConfigure'] == [[1, 0]]


def test_record_reads_every_chunk_in_order_and_stops_the_device():
    scope, lib = recording_scope([(0, 0, 0), (4, 0, 0), (0, 0, 0), (6, 0, 0)],
                                 states=[d.STATE_PREFILL, d.STATE_ARMED])
    stamps = []
    rec = scope.record(now=lambda: stamps.append(len(lib.calls)) or 'T', poll_period_s=0.0)
    assert rec.ch1_v.dtype == np.float32
    np.testing.assert_array_equal(rec.ch1_v, np.arange(10))
    assert (rec.lost_samples, rec.corrupted_samples, rec.start_time) == (0, 0, 'T')
    configures = [c[1][1:] for c in lib.calls if c[0] == 'FDwfAnalogInConfigure']
    assert configures == [[1, 0], [0, 1], [0, 0]]  # applied, started, stopped
    # stamped when the first samples were read, not at the start command
    assert names(lib)[stamps[0] - 1] == 'FDwfAnalogInStatusData'
    assert stamps[0] - 1 == names(lib).index('FDwfAnalogInStatusData')


def test_record_does_not_stamp_a_start_that_never_delivers():
    scope, lib = recording_scope([], states=[d.STATE_ARMED] * 1000)
    stamps = []
    with pytest.raises(d.DwfError, match='no samples'):
        scope.record(now=lambda: stamps.append(1), poll_period_s=0.0, stall_timeout_s=0.05)
    assert stamps == []


def test_record_counts_lost_samples_and_keeps_their_place():
    scope, lib = recording_scope([(3, 0, 0), (4, 2, 1), (5, 0, 0)])
    rec = scope.record(poll_period_s=0.0)
    assert (rec.lost_samples, rec.corrupted_samples) == (2, 1)
    # samples 3 and 4 were lost: zeros there, the rest at their own index
    np.testing.assert_array_equal(rec.ch1_v, [0, 1, 2, 0, 0, 5, 6, 7, 8, 9])


def test_record_takes_only_what_it_needs_of_the_last_chunk():
    scope, lib = recording_scope([(8, 0, 0), (8, 0, 0)])
    rec = scope.record(poll_period_s=0.0)
    np.testing.assert_array_equal(rec.ch1_v, np.arange(10))
    assert [c[1][-1] for c in lib.calls if c[0] == 'FDwfAnalogInStatusData'] == [8, 2]


def test_record_fails_when_the_device_stops_delivering():
    scope, lib = recording_scope([(3, 0, 0)])
    with pytest.raises(d.DwfError, match='no samples'):
        scope.record(poll_period_s=0.0, stall_timeout_s=0.05)
    assert [c[1][1:] for c in lib.calls if c[0] == 'FDwfAnalogInConfigure'][-1] == [0, 0]


def test_record_fails_when_the_device_ends_early():
    scope, lib = recording_scope([(3, 0, 0), (0, 0, 0)],
                                 states=[d.STATE_TRIGGERED, d.STATE_DONE])
    with pytest.raises(d.DwfError, match='ended after 3 of 10'):
        scope.record(poll_period_s=0.0)


def test_record_can_be_aborted():
    scope, lib = recording_scope([(3, 0, 0)] * 2)
    with pytest.raises(RecordingAborted):
        scope.record(should_stop=lambda: len(lib.chunks) == 1, poll_period_s=0.0)
    assert [c[1][1:] for c in lib.calls if c[0] == 'FDwfAnalogInConfigure'][-1] == [0, 0]


def test_record_keeps_the_sample_count_when_the_device_rounds_the_rate():
    lib = RecordLib([])
    lib._FDwfAnalogInFrequencyGet = lambda _h, hz: setattr(hz.contents, 'value', 999.0)
    scope = AnalogDiscoveryScope(dwf=d.Dwf(lib))
    scope.open()
    assert scope.configure_record(RecordSettings(1000.0, 0.01), settle_s=0.0) == 999.0
    assert scope.record_settings.num_samples() == 10
    assert lib.record_length_s == pytest.approx(10 / 999.0)


@pytest.mark.parametrize('kwargs', [dict(sample_rate_hz=0.0), dict(duration_s=-1.0),
                                    dict(duration_s=float('nan')), dict(ch1_range_peak_v=0.0),
                                    dict(sample_rate_hz=1e9), dict(duration_s=1e-9)])
def test_record_settings_validation(kwargs):
    with pytest.raises(ValueError):
        RecordSettings(**kwargs).validate()


def test_record_before_configure_is_refused():
    with pytest.raises(RuntimeError):
        AnalogDiscoveryScope(dwf=d.Dwf(RecordLib([]))).record()


# --- AeRecorder on the mock scope ---

def mock_recorder(tmp_path, realtime=False, duration_s=0.3, lost=0, **kw):
    scope = MockScope(realtime=realtime, amplitude_v=0.2)
    scope.open()
    scope.configure_record(RecordSettings(sample_rate_hz=1e6, duration_s=duration_s))
    scope.lost_samples = lost
    results = []
    rec = AeRecorder(scope, str(tmp_path / 'ae'), str(tmp_path / 'ae.csv'),
                     skip_samples=100000, ros_time_s=lambda: 12.5,
                     on_result=results.append, **kw)
    return rec, scope, results


def test_a_recording_is_saved_with_its_value(tmp_path):
    rec, scope, results = mock_recorder(tmp_path)
    ok, message, _ = rec.start()
    assert ok, message
    ok, result = rec.get(5.0)
    assert ok and result.complete and not result.confirmed and not result.error
    data = np.load(result.file)
    assert data.dtype == np.float32 and len(data) == 300000
    assert result.value_v2 == pytest.approx(spectrum_integral_v2(data, 1e6, 100000, 100e3))
    assert result.value_v2 > 0
    meta = json.load(open(result.file[:-4] + '.json'))
    assert meta['sample_rate_hz'] == 1e6 and meta['num_samples'] == 300000
    assert meta['start_ros_time_s'] == 12.5 and meta['lost_samples'] == 0
    assert meta['feature']['value'] == result.value_v2
    assert meta['feature']['skip_samples'] == 100000
    rows = list(csv.DictReader(open(tmp_path / 'ae.csv')))
    assert len(rows) == 1 and rows[0]['file'] == result.file
    # saved but not valid until the caller confirms it
    assert (rows[0]['complete'], rows[0]['confirmed'], rows[0]['valid']) == ('True', 'False', 'False')
    assert meta['complete'] is True and meta['valid'] is False
    assert results == [result]
    assert 'ae_spectrum_integral=' in result.message()


def test_get_does_not_return_the_same_recording_twice(tmp_path):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    assert rec.get(5.0)[0]
    ok, message = rec.get(1.0)
    assert not ok and 'already read' in message
    assert rec.start()[0] and rec.get(5.0)[0]  # the next cycle
    assert len(list(csv.DictReader(open(tmp_path / 'ae.csv')))) == 2
    assert len(os.listdir(tmp_path / 'ae')) == 4


def test_lost_samples_make_the_value_invalid_but_the_file_is_kept(tmp_path):
    rec, _, results = mock_recorder(tmp_path, lost=5)
    rec.start()
    ok, result = rec.get(5.0)
    assert not ok and not result.complete and '5 samples lost' in result.error
    assert os.path.exists(result.file)
    assert json.load(open(result.file[:-4] + '.json'))['lost_samples'] == 5
    assert list(csv.DictReader(open(tmp_path / 'ae.csv')))[0]['complete'] == 'False'
    ok, message = rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')
    assert not ok and 'not complete' in message
    assert json.load(open(result.file[:-4] + '.json'))['valid'] is False


def test_start_is_refused_while_recording_and_stop_aborts_without_saving(tmp_path):
    rec, _, results = mock_recorder(tmp_path, realtime=True, duration_s=5.0)
    assert rec.start()[0]
    ok, message, _ = rec.start()
    assert not ok and 'still recording' in message
    t0 = time.monotonic()
    assert rec.stop() == (True, 'recording 1 aborted (nothing saved)')
    assert time.monotonic() - t0 < 1.0
    assert not rec.busy() and results == []
    assert os.listdir(tmp_path / 'ae') == []
    assert not rec.get(0.1)[0]


def test_get_times_out_while_the_recording_runs(tmp_path):
    rec, _, _ = mock_recorder(tmp_path, realtime=True, duration_s=1.0)
    rec.start()
    ok, message = rec.get(0.1)
    assert not ok and 'not finished' in message
    assert rec.get(5.0)[0]  # still delivered afterwards


def test_start_is_refused_without_disk_space(tmp_path, monkeypatch):
    rec, _, _ = mock_recorder(tmp_path)
    monkeypatch.setattr(ae_record.shutil, 'disk_usage',
                        lambda path: type('U', (), {'free': 10 * 1024 * 1024})())
    ok, message, _ = rec.start()
    assert not ok and 'GB free' in message and not rec.busy()


def test_a_device_error_is_reported_by_start(tmp_path):
    rec, scope, _ = mock_recorder(tmp_path)

    def broken(**kw):
        raise d.DwfError('FDwfAnalogInConfigure failed: fake')
    scope.record = broken
    ok, message, _ = rec.start()
    assert not ok and 'fake' in message
    ok, result = rec.get(1.0)
    assert not ok and 'fake' in result.error and result.file == ''


def test_files_are_never_overwritten(tmp_path):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    first = rec.get(5.0)[1].file
    rec._index = 0  # e.g. a restart within the same second: same recording_id again
    rec.start()
    ok, result = rec.get(5.0)
    assert not ok and 'not saved' in result.error
    assert os.path.exists(first)


def test_recording_ids_differ_after_a_restart(tmp_path, monkeypatch):
    first, _, _ = mock_recorder(tmp_path)
    second, _, _ = mock_recorder(tmp_path)  # a restart within the same second
    assert first.recording_id(1) != second.recording_id(1)
    assert re.fullmatch(r'ae_\d{8}_\d{6}_[0-9a-f]{8}_0001', first.recording_id(1))


def test_confirm_refuses_another_recording_id(tmp_path):
    rec, _, _ = mock_recorder(tmp_path)
    ok, _, first_id = rec.start()
    rec.get(5.0)
    ok, _, second_id = rec.start()
    ok, second = rec.get(5.0)
    ok, message = rec.confirm(first_id)  # a late confirm of the first recording
    assert not ok and 'not the recording last returned by get' in message
    assert json.load(open(second.file[:-4] + '.json'))['valid'] is False
    assert rec.confirm(second_id)[0]

def test_record_dir_and_skip_are_checked(tmp_path):
    scope = MockScope(realtime=False)
    scope.configure_record(RecordSettings(sample_rate_hz=1e6, duration_s=0.1))
    with pytest.raises(ValueError):
        AeRecorder(scope, '')
    with pytest.raises(ValueError):
        AeRecorder(scope, str(tmp_path), skip_samples=99999)


def test_start_and_stop_from_other_threads_do_not_race(tmp_path):
    rec, _, _ = mock_recorder(tmp_path, realtime=True, duration_s=0.5)
    answers = []
    threads = [threading.Thread(target=lambda: answers.append(rec.start()[0])) for _ in range(5)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert answers.count(True) == 1
    assert rec.get(5.0)[0]


def test_ctypes_pointer_of_a_numpy_chunk_is_written_in_place():
    buf = np.zeros(3)
    ptr = buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    ptr[1] = 2.0
    assert buf[1] == 2.0


def test_frequency_threshold_must_leave_bins(tmp_path):
    scope = MockScope(realtime=False)
    scope.configure_record(RecordSettings(sample_rate_hz=1e6, duration_s=0.3))
    for bad in (float('nan'), -1.0, 5e5, 6e5):
        with pytest.raises(ValueError, match='frequency_threshold_hz'):
            AeRecorder(scope, str(tmp_path), skip_samples=0, frequency_threshold_hz=bad)
    AeRecorder(scope, str(tmp_path), skip_samples=0, frequency_threshold_hz=4.99e5)


def test_get_never_returns_a_newer_recording_than_it_waited_for(tmp_path):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    rec._thread.join()
    real_join = threading.Thread.join

    def join_then_restart(thread, timeout=None):
        real_join(thread, timeout)
        thread.join = lambda timeout=None: None
        rec.start()  # a second start between get's wait and its read
        real_join(rec._thread)
    rec._thread.join = lambda timeout=None: join_then_restart(rec._thread, timeout)
    ok, message = rec.get(5.0)
    assert not ok and 'recording 1' in message and 'replaced' in message


def test_stop_while_saving_reports_the_saved_recording(tmp_path, monkeypatch):
    rec, _, _ = mock_recorder(tmp_path)
    saving = threading.Event()
    release = threading.Event()
    real = ae_record.spectrum_integral_v2

    def slow(*a, **kw):
        saving.set()
        release.wait(5.0)
        return real(*a, **kw)
    monkeypatch.setattr(ae_record, 'spectrum_integral_v2', slow)
    assert rec.start()[0]
    assert saving.wait(5.0)
    answer = []
    t = threading.Thread(target=lambda: answer.append(rec.stop()))
    t.start()
    time.sleep(0.1)
    release.set()
    t.join(5.0)
    ok, message = answer[0]
    assert ok and 'already finished and is saved' in message
    assert len([f for f in os.listdir(tmp_path / 'ae') if f.endswith('.npy')]) == 1


def test_an_aborted_start_is_not_reported_as_started(tmp_path):
    rec, scope, _ = mock_recorder(tmp_path)

    def aborted(**kw):
        raise RecordingAborted('stopped')
    scope.record = aborted
    ok, message, _ = rec.start()
    assert not ok and 'aborted' in message


def test_confirm_makes_the_last_read_recording_valid(tmp_path):
    rec, _, results = mock_recorder(tmp_path)
    assert not rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')[0]  # nothing read yet
    rec.start()
    ok, result = rec.get(5.0)
    assert ok
    ok, message = rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')
    assert ok and 'confirmed' in message
    meta = json.load(open(result.file[:-4] + '.json'))
    assert meta['confirmed'] is True and meta['valid'] is True
    rows = list(csv.DictReader(open(tmp_path / 'ae.csv')))
    assert [(r['index'], r['valid'], r['note']) for r in rows] == [
        ('1', 'False', 'saved'), ('1', 'True', 'confirmed')]
    assert results[-1].confirmed
    ok, message = rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')  # idempotent: no third CSV row
    assert ok and 'already confirmed' in message
    assert len(list(csv.DictReader(open(tmp_path / 'ae.csv')))) == 2


def test_an_unread_or_unconfirmed_recording_stays_not_valid(tmp_path):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    rec._thread.join()  # saved, never read (the caller failed or was stopped)
    assert not rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')[0]
    rec.start()  # the next cycle; the first one stays as it is
    ok, second = rec.get(5.0)
    assert ok and rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')[0]
    metas = sorted(f for f in os.listdir(tmp_path / 'ae') if f.endswith('.json'))
    valid = [json.load(open(tmp_path / 'ae' / f))['valid'] for f in metas]
    assert valid == [False, True]


def test_confirm_is_bound_to_the_recording_read_even_after_a_new_start(tmp_path):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    ok, first = rec.get(5.0)
    rec.start()
    rec._thread.join()
    assert rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')[0]
    assert json.load(open(first.file[:-4] + '.json'))['valid'] is True
    later = rec.get(5.0)[1]
    assert json.load(open(later.file[:-4] + '.json'))['valid'] is False


def test_a_failed_confirmation_can_be_repeated(tmp_path, monkeypatch):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    ok, result = rec.get(5.0)
    rid = result.recording_id
    real = rec._log_row
    monkeypatch.setattr(rec, '_log_row', lambda *a, **k: (_ for _ in ()).throw(OSError('disk full')))
    ok, message = rec.confirm(rid)
    # the .json (authoritative) is confirmed first; only the CSV row is missing
    assert not ok and 'CSV row' in message and 'try again' in message
    assert json.load(open(result.file[:-4] + '.json'))['valid'] is True
    monkeypatch.setattr(rec, '_log_row', real)
    assert rec.confirm(rid)[0]
    rows = list(csv.DictReader(open(tmp_path / 'ae.csv')))
    assert [r['valid'] for r in rows] == ['False', 'True']

def test_an_old_stop_does_not_drop_a_newer_result(tmp_path):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    rec._thread.join()
    real_join = threading.Thread.join

    def join_then_restart(thread, timeout=None):
        real_join(thread, timeout)
        rec.start()  # recording 2 starts and finishes before stop takes the lock again
        real_join(rec._thread)
    rec._phase = 'saving'
    thread = rec._thread
    alive = iter([True])  # stop's first look: recording 1 still being saved; then done
    thread.is_alive = lambda: next(alive, False)
    thread.join = lambda timeout=None: join_then_restart(thread, timeout)
    rec._thread = thread
    rec._result = None
    assert rec.stop()[0]
    ok, result = rec.get(5.0)
    assert ok and result.index == 2


def test_a_failed_csv_row_leaves_the_metadata_not_valid(tmp_path):
    rec, _, _ = mock_recorder(tmp_path)
    rec.feature_log_csv = str(tmp_path / 'missing_dir' / 'ae.csv')
    rec.start()
    ok, result = rec.get(5.0)
    assert not ok and 'CSV row not written' in result.error and result.file
    meta = json.load(open(result.file[:-4] + '.json'))
    assert meta['complete'] is False and any('CSV' in e for e in meta['errors'])
    assert not rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')[0]


def test_configure_record_keeps_the_range_the_device_took():
    lib = RecordLib([])
    lib.range_pp_v = 50.0  # asked +-1 V, the device takes +-25 V
    scope = AnalogDiscoveryScope(dwf=d.Dwf(lib))
    scope.open()
    scope.configure_record(RecordSettings(1000.0, 0.01, ch1_range_peak_v=1.0), settle_s=0.0)
    assert scope.record_settings.ch1_range_peak_v == 25.0


def test_a_concurrent_confirm_waits_for_the_first_to_be_written(tmp_path, monkeypatch):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    rec.get(5.0)
    writing = threading.Event()
    release = threading.Event()
    real = rec._log_row

    def slow_then_fail(*a, **k):
        writing.set()
        release.wait(5.0)
        raise OSError('disk full')
    monkeypatch.setattr(rec, '_log_row', slow_then_fail)
    first = []
    t = threading.Thread(target=lambda: first.append(rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')))
    t.start()
    assert writing.wait(5.0)
    second = []
    t2 = threading.Thread(target=lambda: second.append(rec.confirm(rec._last_read.recording_id if rec._last_read else 'x')))
    t2.start()
    time.sleep(0.1)
    assert second == []  # not answered 'already confirmed' while the first is writing
    monkeypatch.setattr(rec, '_log_row', real)
    release.set()
    t.join(5.0)
    t2.join(5.0)
    assert not first[0][0] and second[0][0]
    assert json.load(open(rec._last_read.file[:-4] + '.json'))['valid'] is True


def test_a_failed_metadata_confirmation_leaves_both_not_valid(tmp_path, monkeypatch):
    rec, _, _ = mock_recorder(tmp_path)
    rec.start()
    ok, result = rec.get(5.0)
    real = rec._write_meta
    monkeypatch.setattr(rec, '_write_meta', lambda *a: (_ for _ in ()).throw(OSError('disk full')))
    assert not rec.confirm(result.recording_id)[0]
    rows = list(csv.DictReader(open(tmp_path / 'ae.csv')))
    assert [r['valid'] for r in rows] == ['False']  # no valid row before the .json says so
    assert json.load(open(result.file[:-4] + '.json'))['valid'] is False
    monkeypatch.setattr(rec, '_write_meta', real)
    assert rec.confirm(result.recording_id)[0]
    assert json.load(open(result.file[:-4] + '.json'))['valid'] is True


def test_a_timed_out_start_only_stops_its_own_recording(tmp_path):
    rec, scope, _ = mock_recorder(tmp_path)
    real = scope.record
    gate = threading.Event()

    def slow_start(**kw):
        gate.wait(5.0)  # the first samples come late
        return real(**kw)
    scope.record = slow_start
    ok, message, _ = rec.start(timeout_s=0.05)
    assert not ok
    gate.set()
    rec._thread.join(5.0)  # the late recording is aborted, nothing saved
    scope.record = real
    ok, _, rid = rec.start()
    assert ok
    ok, result = rec.get(5.0)
    assert ok and result.recording_id == rid  # not stopped by the first start's timeout

def test_a_stop_right_after_the_last_read_saves_nothing(tmp_path):
    rec, scope, _ = mock_recorder(tmp_path)
    real = scope.record

    def record_then_stop(**kw):
        rec_ = real(**kw)
        rec._stop.set()  # stop() took the lock after the read loop, before the save
        return rec_
    scope.record = record_then_stop
    ok, message, _ = rec.start()
    rec._thread.join()
    assert os.listdir(tmp_path / 'ae') == []
