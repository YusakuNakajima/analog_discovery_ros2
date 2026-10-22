import numpy as np
from analog_discovery.mock_scope import MockScope
from analog_discovery.scope import CaptureSettings


class Clock:
    t_s = 0.0

    def __call__(self):
        return self.t_s


def test_impulse_starts_at_the_trigger_index():
    clock = Clock()
    scope = MockScope(period_s=0.1, noise_v=0.0, clock=clock)
    scope.open()
    scope.configure(CaptureSettings(buffer_samples=1000, pretrigger_samples=200))
    scope.arm()
    assert scope.poll() is None
    clock.t_s = 0.2
    cap = scope.poll()
    assert cap.trigger_index == 200
    assert np.all(cap.ch1_v[:200] == 0.0)
    assert cap.ch1_v[200] == 1.0 and cap.ch2_v[200] == 0.5
    assert scope.poll() is None  # needs a new arm()
