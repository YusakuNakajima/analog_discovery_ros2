#!/usr/bin/env python3
"""Record the AE signal continuously on request, one recording per grinding cycle (ROS 1 style).

Service ~/command (analog_discovery_ros2/srv/AeRecordCommand), command:
    start    start a recording of duration_s at sample_rate_hz on CH1 (Record mode);
             answers once samples arrive, with its recording_id. Refused while one runs.
    get      wait (up to duration_s + get_margin_s) until the last started recording is
             saved; answers its recording_id, ae_spectrum_integral_v2 (ROS 1 formula, V^2)
             and file. success is false when samples were lost, the file could not be
             written, etc.
    confirm  (recording_id = the one get returned) the caller's cycle went as planned:
             only then its .json says valid true and a CSV row valid True is added.
             Unconfirmed recordings stay on disk but are not valid. Retryable.
    stop     abort a recording still being recorded (nothing saved) / drop an unread
             result; one already being saved is not aborted (the message says so).
Topic:
    ~/ae_spectrum_integral_v2  std_msgs/Float64, for each confirmed recording (rosbag)

Each recording: <record_dir>/<recording_id>.npy (float32, V) and .json, recording_id =
ae_<node start YYYYmmdd_HHMMSS>_<index>
(rate, start time, lost samples, value). The device is opened at start-up and kept;
the triggered-capture node (analog_discovery_node.py) cannot run at the same time.
For looking at and recording data; nothing here commands a robot.
"""

import math
import signal

import rclpy
from analog_discovery.ae_record import AeRecorder, ROS1_FREQUENCY_THRESHOLD_HZ, ROS1_SKIP_SAMPLES
from analog_discovery.dwf import DwfError
from analog_discovery.scope import AnalogDiscoveryScope, DEFAULT_SETTLE_S, RecordSettings
from analog_discovery_ros2.srv import AeRecordCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64


def create_scope(backend, serial, config_index):
    if backend == 'dwf':
        return AnalogDiscoveryScope(serial=serial, config_index=config_index)
    if backend == 'mock':
        from analog_discovery.mock_scope import MockScope
        return MockScope(serial=serial, config_index=config_index)
    raise ValueError(f"backend must be 'dwf' or 'mock', got {backend!r}")


class AeRecordNode(Node):

    def __init__(self):
        super().__init__('ae_recorder')
        p = self.declare_parameter
        backend = p('backend', 'dwf').value  # 'dwf' (WaveForms SDK) or 'mock'
        serial = p('serial', '').value  # '' = first free AD3/AD2
        config_index = p('config_index', -1).value  # -1 = largest scope buffer
        settle_s = p('settle_s', DEFAULT_SETTLE_S).value  # s, wait after configuring
        defaults = RecordSettings()
        settings = RecordSettings(**{
            field: p(field, getattr(defaults, field)).value
            for field in RecordSettings.__dataclass_fields__})
        record_dir = p('record_dir', '').value
        feature_log_csv = p('feature_log_csv', '').value
        skip_samples = p('skip_samples', ROS1_SKIP_SAMPLES).value
        frequency_threshold_hz = p('frequency_threshold_hz', ROS1_FREQUENCY_THRESHOLD_HZ).value
        self.start_timeout_s = p('start_timeout_s', 2.0).value  # s
        # s, beyond the recording itself: saving + the FFT of 24 M samples take a few s
        self.get_margin_s = p('get_margin_s', 20.0).value
        settings.validate()
        for name, value in (('start_timeout_s', self.start_timeout_s),
                            ('get_margin_s', self.get_margin_s)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be positive')
        if not record_dir:
            raise ValueError('record_dir must be set (directory for the raw AE recordings)')

        self.pub = self.create_publisher(Float64, '~/ae_spectrum_integral_v2', 10)
        self.scope = create_scope(backend, serial, config_index)
        self.scope.open()
        try:
            self.scope.configure_record(settings, settle_s)
            s = self.scope.record_settings
            self.recorder = AeRecorder(
                self.scope, record_dir, feature_log_csv, skip_samples, frequency_threshold_hz,
                ros_time_s=lambda: self.get_clock().now().nanoseconds * 1e-9,
                log=self.get_logger().info, on_result=self._on_result,
                extra_metadata={'backend': backend})
        except BaseException:
            self.scope.close()
            raise
        self.create_service(AeRecordCommand, '~/command', self._command,
                            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            f'{self.scope.device_name}: CH1 +-{s.ch1_range_peak_v:g} V, {s.num_samples()} samples '
            f'at {s.sample_rate_hz:g} Hz ({s.duration_s:g} s) per recording -> {record_dir}'
            + (f', values -> {feature_log_csv}' if feature_log_csv else ''))

    def _on_result(self, result):
        if result.confirmed:
            self.pub.publish(Float64(data=result.value_v2))

    def _command(self, req, res):
        res.ae_spectrum_integral_v2 = math.nan
        if req.command == 'start':
            res.success, res.message, res.recording_id = self.recorder.start(self.start_timeout_s)
        elif req.command == 'get':
            ok, result = self.recorder.get(self.scope.record_settings.duration_s + self.get_margin_s)
            if isinstance(result, str):
                res.success, res.message = False, result
            else:
                res.success, res.message = ok, result.message()
                res.recording_id, res.file = result.recording_id, result.file
                res.ae_spectrum_integral_v2 = result.value_v2
        elif req.command == 'confirm':
            res.success, res.message = self.recorder.confirm(req.recording_id)
            res.recording_id = req.recording_id
        elif req.command == 'stop':
            res.success, res.message = self.recorder.stop()
        else:
            res.success, res.message = False, f'unknown command {req.command!r} (start|get|confirm|stop)'
        if res.success:  # rclpy: one call site must keep one severity
            self.get_logger().info(f'{req.command}: {res.message}')
        else:
            self.get_logger().error(f'{req.command}: {res.message}')
        return res

    def shutdown(self):
        # aborts a recording still being recorded; waits for one being saved
        ok, message = self.recorder.stop(
            timeout_s=self.scope.record_settings.duration_s + self.get_margin_s)
        if not ok:
            self.get_logger().error(f'shutdown: {message}; waiting for it to finish')
            self.recorder.join()  # a save still running would be cut off by the exit
        self.scope.close()


def main():
    rclpy.init()
    node = None
    try:
        node = AeRecordNode()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
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
