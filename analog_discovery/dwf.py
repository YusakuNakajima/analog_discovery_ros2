"""Thin ctypes binding to the Digilent WaveForms SDK (libdwf).

Only the calls this package uses are declared. Constant values are taken from
the WaveForms SDK Reference Manual (dwfsdk.pdf, revised 2025-04-08) rather than
copied from the vendor's dwfconstants.py, whose license forbids modification.

Every FDwf* function returns 0 on failure; call() turns that into DwfError
with the SDK's last error message.
"""

import ctypes
import sys

# --- Device enumeration ---
ENUMFILTER_ALL = 0
DEVID_DISCOVERY2 = 3
DEVID_DISCOVERY3 = 10
DEVICE_NAMES = {DEVID_DISCOVERY2: 'Analog Discovery 2', DEVID_DISCOVERY3: 'Analog Discovery 3'}
HDWF_NONE = 0
DECI_ANALOG_IN_BUFFER_SIZE = 7  # FDwfEnumConfigInfo: scope buffer per channel

# --- Analog in ---
ACQMODE_SINGLE = 0
TRIGSRC_NONE = 0
TRIGSRC_DETECTOR_ANALOG_IN = 2
TRIGTYPE_EDGE = 0
TRIGGER_SLOPES = {'rising': 0, 'falling': 1, 'either': 2}  # DwfTriggerSlope

# DwfState (FDwfAnalogInStatus)
STATE_READY = 0
STATE_ARMED = 1
STATE_DONE = 2
STATE_TRIGGERED = 3
STATE_CONFIG = 4
STATE_PREFILL = 5
STATE_WAIT = 7

# --- Analog out (selftest only) ---
ANALOG_OUT_NODE_CARRIER = 0
FUNC_SQUARE = 2


class DwfError(RuntimeError):
    """A WaveForms SDK call failed."""


def load_library():
    """Load libdwf, or raise DwfError with installation advice."""
    if sys.platform.startswith('win'):
        name = 'dwf'
    elif sys.platform.startswith('darwin'):
        name = '/Library/Frameworks/dwf.framework/dwf'
    else:
        name = 'libdwf.so'
    try:
        return ctypes.cdll.LoadLibrary(name)
    except OSError as e:
        raise DwfError(
            f'cannot load {name} ({e}); install Digilent Adept Runtime and WaveForms '
            '(scripts/install_waveforms.sh)') from e


class Dwf:
    """libdwf with error checking. lib is injectable for tests."""

    def __init__(self, lib=None):
        self.lib = load_library() if lib is None else lib

    def call(self, name, *args):
        if not getattr(self.lib, name)(*args):
            raise DwfError(f'{name} failed: {self.last_error()}')

    def last_error(self):
        buf = ctypes.create_string_buffer(512)
        self.lib.FDwfGetLastErrorMsg(buf)
        return buf.value.decode(errors='replace').strip() or 'no message'

    def version(self):
        buf = ctypes.create_string_buffer(32)
        self.call('FDwfGetVersion', buf)
        return buf.value.decode()
