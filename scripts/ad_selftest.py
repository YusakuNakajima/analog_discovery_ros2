#!/usr/bin/env python3
"""Check the Analog Discovery scope path on a real device.

    ad_selftest.py                    # one auto-triggered capture; no outputs touched
    ad_selftest.py --loopback         # W1 drives a square wave into CH1 (see below)

--loopback needs a BNC cable from W1 to CH1 (BNC Adapter Board) and NOTHING
else on W1 or CH1: W1 outputs a 1 kHz square wave of +-0.5 V. It checks that
the rising edge lands at the requested pretrigger index, which verifies the
trigger-position convention assumed in scope.py.
"""

import argparse
import ctypes
import sys
import time

import numpy as np

from analog_discovery import dwf as d
from analog_discovery.scope import AnalogDiscoveryScope, CaptureSettings

SQUARE_FREQUENCY_HZ = 1e3
SQUARE_AMPLITUDE_V = 0.5


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
                crossing = int(np.argmax((v[1:] > 0.0) & (v[:-1] <= 0.0))) + 1
                ok &= check(capture.trigger_index == pre,
                            f'loopback: reported trigger_index {capture.trigger_index} '
                            f'(requested {pre})')
                ok &= check(abs(crossing - pre) <= 3,
                            f'loopback: first rising crossing at sample {crossing} '
                            f'(expected {pre})')
                ok &= check(0.4 < v.max() < 0.6 and -0.6 < v.min() < -0.4,
                            f'loopback: CH1 {v.min():+.3f}..{v.max():+.3f} V '
                            f'(W1 +-{SQUARE_AMPLITUDE_V} V)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
