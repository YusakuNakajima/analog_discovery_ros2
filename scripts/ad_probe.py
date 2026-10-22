#!/usr/bin/env python3
"""List attached Analog Discovery devices and their scope buffer per configuration.

Nothing is opened or configured; only the WaveForms SDK enumeration is used.
"""

from analog_discovery.dwf import DwfError
from analog_discovery.scope import AnalogDiscoveryScope


def main():
    scope = AnalogDiscoveryScope()
    try:
        print(f'WaveForms SDK {scope.dwf.version()}')
        devices = scope.list_devices()
    except DwfError as e:
        raise SystemExit(str(e))
    print(f'{len(devices)} Analog Discovery device(s)')
    for dev in devices:
        state = 'IN USE' if dev['in_use'] else 'free'
        sizes = scope._config_buffers(dev['index'])
        print(f'  [{dev["index"]}] {dev["name"]}  serial {dev["serial"]}  {state}')
        print('      scope buffer per configuration (samples/ch): '
              + ', '.join(f'{i}: {n}' for i, n in enumerate(sizes)))


if __name__ == '__main__':
    main()
