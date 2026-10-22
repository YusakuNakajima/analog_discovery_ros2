#!/usr/bin/env python3
"""Publish triggered Analog Discovery captures as analog_discovery_ros2/msg/ScopeCapture.

Each trigger yields one capture of buffer_samples per channel, pretrigger_samples
of them before the trigger, on ~/capture. For quick plotting the signed sample
of largest magnitude of each capture is also published as std_msgs/Float32 on
~/ch1_peak_v and ~/ch2_peak_v (V).

The device is re-armed after each capture is read, so triggers during the
read-out are missed. The node exits on any SDK error rather than publishing
data it cannot vouch for.
"""

import array
import signal

import numpy as np
import rclpy
from analog_discovery.dwf import DwfError
from analog_discovery.scope import AnalogDiscoveryScope, CaptureSettings
from analog_discovery_ros2.msg import ScopeCapture
from rclpy.node import Node
from std_msgs.msg import Float32


def signed_peak_v(samples_v):
    """The sample of largest magnitude, keeping its sign (V)."""
    return float(samples_v[np.argmax(np.abs(samples_v))])


def create_scope(backend, serial, config_index):
    if backend == 'dwf':
        return AnalogDiscoveryScope(serial=serial, config_index=config_index)
    if backend == 'mock':
        from analog_discovery.mock_scope import MockScope
        return MockScope(serial=serial, config_index=config_index)
    raise ValueError(f"backend must be 'dwf' or 'mock', got {backend!r}")


class AnalogDiscoveryNode(Node):

    def __init__(self):
        super().__init__('analog_discovery')
        p = self.declare_parameter
        backend = p('backend', 'dwf').value  # 'dwf' (WaveForms SDK) or 'mock'
        serial = p('serial', '').value  # '' = first free AD3/AD2
        config_index = p('config_index', -1).value  # -1 = largest scope buffer
        self.frame_id = p('frame_id', 'analog_discovery').value
        self.poll_period_s = p('poll_period_s', 0.005).value
        defaults = CaptureSettings()
        self.settings = CaptureSettings(**{
            field: p(field, getattr(defaults, field)).value
            for field in CaptureSettings.__dataclass_fields__})
        # Checked before the device is opened.
        self.settings.validate()
        if not self.poll_period_s > 0.0:
            raise ValueError('poll_period_s must be positive')

        self.pub = self.create_publisher(ScopeCapture, '~/capture', 10)
        self.peak_pubs = (self.create_publisher(Float32, '~/ch1_peak_v', 10),
                          self.create_publisher(Float32, '~/ch2_peak_v', 10))
        self.captures = 0
        self.scope = create_scope(backend, serial, config_index)
        self.scope.open()
        try:
            self.sample_rate_hz = self.scope.configure(self.settings)
            self.scope.arm()
            self.timer = self.create_timer(self.poll_period_s, self.poll)
        except BaseException:
            self.scope.close()
            raise
        s = self.scope.settings
        self.get_logger().info(
            f'{self.scope.device_name}: {s.buffer_samples} samples at '
            f'{self.sample_rate_hz:g} Hz ({s.buffer_samples / self.sample_rate_hz * 1e3:.3g} ms) '
            f'per capture, trigger {s.trigger_source} {s.trigger_slope} '
            f'{s.trigger_level_v:g} V, {s.pretrigger_samples} pretrigger samples')

    def poll(self):
        capture = self.scope.poll()
        if capture is None:
            return
        now = self.get_clock().now()
        self.scope.arm()  # re-arm first to shorten the dead time
        self.publish(capture, now)

    def publish(self, capture, seen_time):
        msg = ScopeCapture()
        post_trigger_ns = round((len(capture.ch1_v) - capture.trigger_index) * 1e9
                                / capture.sample_rate_hz)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            seen_time.nanoseconds - post_trigger_ns, 1_000_000_000)
        msg.header.frame_id = self.frame_id
        msg.capture_index = self.captures
        msg.sample_rate_hz = capture.sample_rate_hz
        msg.trigger_index = capture.trigger_index
        msg.auto_triggered = capture.auto_triggered
        msg.ch1_v = array.array('f', capture.ch1_v.astype(np.float32).tobytes())
        msg.ch2_v = array.array('f', capture.ch2_v.astype(np.float32).tobytes())
        (msg.device_trigger_sec_utc, msg.device_trigger_tick,
         msg.device_ticks_per_second) = capture.device_time
        self.pub.publish(msg)
        for pub, samples_v in zip(self.peak_pubs, (capture.ch1_v, capture.ch2_v)):
            if len(samples_v):
                pub.publish(Float32(data=signed_peak_v(samples_v)))
        self.captures += 1

    def shutdown(self):
        self.scope.close()


def main():
    rclpy.init()
    node = None
    try:
        node = AnalogDiscoveryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except DwfError as e:
        if node is not None:
            node.get_logger().error(f'WaveForms SDK error, exiting: {e}')
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # ros2 launch sends Ctrl-C twice
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
