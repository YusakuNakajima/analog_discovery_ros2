#!/usr/bin/env python3
"""Compute acoustic-emission features from ScopeCapture and publish them.

    ~/hit       analog_discovery_ros2/msg/AeHit       one per triggered capture with a hit
    ~/activity  analog_discovery_ros2/msg/AeActivity  every activity_period_s, over window_s

For looking at the data and recording it; nothing here commands a robot.
Auto-triggered captures (no real event) are skipped.
"""

import math
import signal

import rclpy
from analog_discovery.ae_features import ActivityWindow, hit_features
from analog_discovery_ros2.msg import AeActivity, AeHit, ScopeCapture
from rclpy.node import Node


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class AeFeaturesNode(Node):

    def __init__(self):
        super().__init__('ae_features')
        p = self.declare_parameter
        input_topic = p('input_topic', '/analog_discovery/capture').value
        self.channel = p('channel', 'ch1').value  # 'ch1' | 'ch2'
        # V; default = the driver's default trigger level, so every capture yields a hit
        self.threshold_v = p('threshold_v', 0.02).value
        window_s = p('window_s', 1.0).value
        period_s = p('activity_period_s', 0.1).value
        if self.channel not in ('ch1', 'ch2'):
            raise ValueError("channel must be 'ch1' or 'ch2'")
        for name, value in (('threshold_v', self.threshold_v), ('window_s', window_s),
                            ('activity_period_s', period_s)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be positive')
        self.window = ActivityWindow(window_s)
        self.hit_pub = self.create_publisher(AeHit, '~/hit', 100)
        self.activity_pub = self.create_publisher(AeActivity, '~/activity', 10)
        self.create_subscription(ScopeCapture, input_topic, self.on_capture, 100)
        self.create_timer(period_s, self.publish_activity)

    def on_capture(self, capture):
        if capture.auto_triggered:
            return
        samples_v = capture.ch1_v if self.channel == 'ch1' else capture.ch2_v
        if len(samples_v) == 0:
            return
        f = hit_features(samples_v, capture.sample_rate_hz, capture.trigger_index,
                         self.threshold_v)
        if f is None:
            return
        msg = AeHit(header=capture.header, capture_index=capture.capture_index,
                    threshold_v=self.threshold_v, peak_amplitude_v=f.peak_amplitude_v,
                    rise_time_s=f.rise_time_s, duration_s=f.duration_s, counts=f.counts,
                    energy_v2s=f.energy_v2s, rms_v=f.rms_v,
                    peak_frequency_hz=f.peak_frequency_hz,
                    centroid_frequency_hz=f.centroid_frequency_hz,
                    noise_rms_v=f.noise_rms_v, truncated=f.truncated)
        self.hit_pub.publish(msg)
        self.window.add(stamp_s(capture.header.stamp), f)

    def publish_activity(self):
        now = self.get_clock().now()
        s = self.window.summary(now.nanoseconds * 1e-9)
        msg = AeActivity(window_s=s['window_s'], hits=s['hits'], hit_rate_hz=s['hit_rate_hz'],
                         energy_rate_v2=s['energy_rate_v2'], counts_rate_hz=s['counts_rate_hz'],
                         mean_peak_amplitude_v=s['mean_peak_amplitude_v'],
                         max_peak_amplitude_v=s['max_peak_amplitude_v'],
                         truncated_hits=s['truncated_hits'])
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'analog_discovery'
        self.activity_pub.publish(msg)


def main():
    rclpy.init()
    node = AeFeaturesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # ros2 launch sends Ctrl-C twice
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
