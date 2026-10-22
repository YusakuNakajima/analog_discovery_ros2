#!/usr/bin/env python3
"""Check the Analog Discovery scope path on a real device.

    ad_selftest.py                    # one auto-triggered capture; no outputs touched
    ad_selftest.py --loopback         # W1 drives a square wave into CH1 (see below)

--loopback needs a BNC cable from W1 to CH1 (BNC Adapter Board) and NOTHING
else on W1 or CH1: W1 outputs a 50 Hz square wave of +-0.5 V. Its 20 ms period
is longer than the 8.192 ms capture, so the capture holds exactly one rising
edge; the test checks that this edge lands at the requested pretrigger index,
which verifies the trigger-position convention assumed in scope.py (a wave
that rose several times per capture would hide a shift of whole periods).
"""

import argparse
import ctypes
import sys
import time

import numpy as np

from analog_discovery import dwf as d
from analog_discovery.scope import AnalogDiscoveryScope, CaptureSettings

SQUARE_FREQUENCY_HZ = 50.0  # period 20 ms > capture length 8.192 ms
SQUARE_AMPLITUDE_V = 0.5


def rising_edges(v, low_v=-0.25, high_v=0.25):
    """Sample index of each rise from below low_v to above high_v, at the 0 V crossing."""
    edges = []
    armed = False
    for i, x in enumerate(v):
        if x < low_v:
            armed = True
        elif armed and x > high_v:
            j = i
            while j > 0 and v[j - 1] > 0.0:
                j -= 1
            edges.append(j)
            armed = False
    return edges


def loopback_edge_ok(v, pre, tolerance=3):
    """(ok, edges): exactly one rising edge, within tolerance samples of pre."""
    edges = rising_edges(v)
    return len(edges) == 1 and abs(edges[0] - pre) <= tolerance, edges


def check(ok, text):
    print(('PASS ' if ok else 'FAIL ') + text)
    return ok


def wait_capture(scope, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        capture = scope.poll()
        if capture is not None:
            return capture
        time.sleep(0.002)
    return None


def start_square(scope):
    h, call, c = scope._hdwf, scope.dwf.call, ctypes
    node = c.c_int(d.ANALOG_OUT_NODE_CARRIER)
    call('FDwfAnalogOutNodeEnableSet', h, c.c_int(0), node, c.c_int(1))
    call('FDwfAnalogOutNodeFunctionSet', h, c.c_int(0), node, c.c_ubyte(d.FUNC_SQUARE))
    call('FDwfAnalogOutNodeFrequencySet', h, c.c_int(0), node, c.c_double(SQUARE_FREQUENCY_HZ))
    call('FDwfAnalogOutNodeAmplitudeSet', h, c.c_int(0), node, c.c_double(SQUARE_AMPLITUDE_V))
    call('FDwfAnalogOutNodeOffsetSet', h, c.c_int(0), node, c.c_double(0.0))
    call('FDwfAnalogOutConfigure', h, c.c_int(0), c.c_int(1))


def stop_square(scope):
    scope.dwf.lib.FDwfAnalogOutConfigure(scope._hdwf, ctypes.c_int(0), ctypes.c_int(0))
    scope.dwf.lib.FDwfAnalogOutReset(scope._hdwf, ctypes.c_int(0))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--serial', default='')
    parser.add_argument('--loopback', action='store_true',
                        help='drive W1 (needs a W1 -> CH1 cable, nothing else connected)')
    args = parser.parse_args()
    ok = True
    with AnalogDiscoveryScope(serial=args.serial) as scope:
        print(f'opened {scope.device_name}, buffer up to {scope.max_buffer_samples} samples/ch')
        auto = CaptureSettings(sample_rate_hz=1e6, buffer_samples=8192,
                               trigger_source='none', auto_timeout_s=0.1,
                               pretrigger_samples=0)
        rate_hz = scope.configure(auto)
        ok &= check(abs(rate_hz - 1e6) < 1.0, f'sample rate reads back {rate_hz:g} Hz')
        scope.arm()
        capture = wait_capture(scope, 2.0)
        ok &= check(capture is not None, 'auto-triggered capture arrived')
        if capture is not None:
            for name, v in (('ch1', capture.ch1_v), ('ch2', capture.ch2_v)):
                ok &= check(len(v) == 8192 and np.all(np.isfinite(v)),
                            f'{name}: {len(v)} finite samples, mean {v.mean() * 1e3:+.2f} mV, '
                            f'std {v.std() * 1e3:.3f} mV')
            print(f'device time {capture.device_time}')

        if args.loopback:
            pre = 2000
            edge = CaptureSettings(sample_rate_hz=1e6, buffer_samples=8192, ch2_enabled=False,
                                   trigger_source='ch1', trigger_slope='rising',
                                   trigger_level_v=0.0, trigger_hysteresis_v=0.05,
                                   pretrigger_samples=pre, auto_timeout_s=0.0)
            scope.configure(edge)
            try:
                start_square(scope)
                time.sleep(0.05)
                scope.arm()
                capture = wait_capture(scope, 2.0)
            finally:
                stop_square(scope)
            ok &= check(capture is not None and not capture.auto_triggered,
                        'loopback: triggered on the W1 square wave')
            if capture is not None:
                v = capture.ch1_v
                edge_ok, edges = loopback_edge_ok(v, pre)
                ok &= check(capture.trigger_index == pre,
                            f'loopback: reported trigger_index {capture.trigger_index} '
                            f'(requested {pre})')
                ok &= check(edge_ok, f'loopback: rising edges at {edges} '
                            f'(expected exactly one, at {pre})')
                ok &= check(0.4 < v.max() < 0.6 and -0.6 < v.min() < -0.4,
                            f'loopback: CH1 {v.min():+.3f}..{v.max():+.3f} V '
                            f'(W1 +-{SQUARE_AMPLITUDE_V} V)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
