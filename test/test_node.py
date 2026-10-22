"""The node checks its settings before opening the device and stamps the trigger time."""

import importlib.util
import os

import numpy as np
import pytest
import rclpy
import rclpy.time
from analog_discovery.scope import Capture
from rclpy.parameter import Parameter

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'analog_discovery_node.py')


@pytest.fixture(scope='module')
def mod():
    spec = importlib.util.spec_from_file_location('analog_discovery_node', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    yield module
    rclpy.shutdown()


def with_params(monkeypatch, **params):
    overrides = [Parameter(k, value=v) for k, v in {'backend': 'mock', **params}.items()]
    original = rclpy.node.Node.__init__
    monkeypatch.setattr(rclpy.node.Node, '__init__',
                        lambda self, name, **kw: original(self, name,
                                                          parameter_overrides=overrides, **kw))


@pytest.mark.parametrize('params', [
    {'trigger_level_v': 3.0},
    {'pretrigger_samples': 40000},          # more than the 32768-sample buffer
    {'trigger_source': 'none'},
    {'poll_period_s': 0.0},
    {'sample_rate_hz': -1.0},
    {'settle_s': -0.1},
    {'poll_period_s': float('nan')},
])
def test_bad_settings_are_rejected_before_open(mod, monkeypatch, params):
    opened = []
    monkeypatch.setattr(mod, 'create_scope', lambda *a: opened.append(a))
    with_params(monkeypatch, **params)
    with pytest.raises(ValueError):
        mod.AnalogDiscoveryNode()
    assert opened == []


def test_mock_node_starts_and_closes(mod, monkeypatch):
    with_params(monkeypatch, settle_s=0.0)
    node = mod.AnalogDiscoveryNode()
    assert node.sample_rate_hz == 10e6
    node.shutdown()
    node.destroy_node()


class Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def test_publish_stamps_the_trigger_and_signed_peaks(mod):
    node = object.__new__(mod.AnalogDiscoveryNode)
    node.frame_id = 'f'
    node.captures = 0
    node.pub, node.peak_pubs = Pub(), (Pub(), Pub())
    ch1 = np.zeros(1000)
    ch1[300] = -0.8
    cap = Capture(sample_rate_hz=1e6, trigger_index=200, auto_triggered=False,
                  ch1_v=ch1, ch2_v=np.zeros(0), device_time=(1, 2, 3))
    node.publish(cap, rclpy.time.Time(nanoseconds=5_000_000_000))
    msg = node.pub.msgs[0]
    # 800 post-trigger samples at 1 MHz = 0.8 ms before the time the capture was seen
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (4, 999_200_000)
    assert msg.trigger_index == 200 and len(msg.ch1_v) == 1000 and len(msg.ch2_v) == 0
    assert (msg.device_trigger_sec_utc, msg.device_trigger_tick,
            msg.device_ticks_per_second) == (1, 2, 3)
    assert node.peak_pubs[0].msgs[0].data == pytest.approx(-0.8)
    assert node.peak_pubs[1].msgs == []  # ch2 disabled
    assert node.captures == 1
