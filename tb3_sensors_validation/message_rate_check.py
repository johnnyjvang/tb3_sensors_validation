"""
message_rate_check.py

Subscribe to the main TurtleBot3 sensor topics and measure how consistently
messages arrive over a fixed observation window.

Goal:
- Confirm /scan, /imu, and /odom are publishing
- Measure average message rate for each topic
- Measure min/max time between messages
- Save each topic result to the JSON results file
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
TEST_DURATION = 10.0   # seconds to observe topic message timing

SCAN_TOPIC = '/scan'
IMU_TOPIC = '/imu'
ODOM_TOPIC = '/odom'

# Expected topic rates
SCAN_EXPECTED_RATE = 5.0
IMU_EXPECTED_RATE = 200.0
ODOM_EXPECTED_RATE = 30.0

# Acceptable absolute tolerance in Hz
SCAN_RATE_TOL = 1.0
IMU_RATE_TOL = 15.0
ODOM_RATE_TOL = 5.0


class TopicRateTracker:
    """
    Helper object to track timing statistics for one topic.

    Stores:
    - first message time
    - last message time
    - total message count
    - list of dt values between consecutive messages
    """

    def __init__(self, topic_name, expected_rate, rate_tol):
        self.topic_name = topic_name
        self.expected_rate = expected_rate
        self.rate_tol = rate_tol

        self.first_msg_time = None
        self.last_msg_time = None
        self.msg_count = 0
        self.dt_list = []

    def record_message(self):
        """
        Record that one message has arrived right now.
        """
        now = time.time()

        if self.first_msg_time is None:
            self.first_msg_time = now

        if self.last_msg_time is not None:
            dt = now - self.last_msg_time
            self.dt_list.append(dt)

        self.last_msg_time = now
        self.msg_count += 1

    def estimated_rate(self):
        """
        Return a live estimated rate while the test is still running.
        """
        if self.msg_count < 2 or self.first_msg_time is None or self.last_msg_time is None:
            return 0.0

        active_time = self.last_msg_time - self.first_msg_time
        if active_time <= 0.0:
            return 0.0

        return len(self.dt_list) / active_time

    def finalize(self):
        """
        Compute final statistics for this topic.

        Returns a dictionary containing:
        - status
        - measurement
        - notes
        - detailed numeric stats
        """
        if self.msg_count < 2 or len(self.dt_list) == 0:
            return {
                'status': 'FAIL',
                'measurement': '0.00 Hz',
                'notes': f'No sufficient messages received on {self.topic_name}',
                'avg_rate': 0.0,
                'avg_dt': 0.0,
                'min_dt': 0.0,
                'max_dt': 0.0,
                'elapsed': 0.0,
                'msg_count': self.msg_count,
            }

        elapsed = self.last_msg_time - self.first_msg_time
        avg_rate = len(self.dt_list) / elapsed if elapsed > 0.0 else 0.0
        avg_dt = sum(self.dt_list) / len(self.dt_list)
        min_dt = min(self.dt_list)
        max_dt = max(self.dt_list)

        rate_error = abs(avg_rate - self.expected_rate)
        status = 'PASS' if rate_error <= self.rate_tol else 'FAIL'

        measurement = f'{avg_rate:.2f} Hz'
        notes = (
            f'topic={self.topic_name}, '
            f'expected={self.expected_rate:.1f}Hz, '
            f'rate={avg_rate:.2f}Hz, '
            f'dt_avg={avg_dt:.4f}s, '
            f'dt_min={min_dt:.4f}s, '
            f'dt_max={max_dt:.4f}s'
        )

        return {
            'status': status,
            'measurement': measurement,
            'notes': notes,
            'avg_rate': avg_rate,
            'avg_dt': avg_dt,
            'min_dt': min_dt,
            'max_dt': max_dt,
            'elapsed': elapsed,
            'msg_count': self.msg_count,
        }


class MessageRateCheck(Node):
    def __init__(self):
        super().__init__('message_rate_check')

        # ===== Topic trackers =====
        self.scan_tracker = TopicRateTracker(
            topic_name=SCAN_TOPIC,
            expected_rate=SCAN_EXPECTED_RATE,
            rate_tol=SCAN_RATE_TOL
        )

        self.imu_tracker = TopicRateTracker(
            topic_name=IMU_TOPIC,
            expected_rate=IMU_EXPECTED_RATE,
            rate_tol=IMU_RATE_TOL
        )

        self.odom_tracker = TopicRateTracker(
            topic_name=ODOM_TOPIC,
            expected_rate=ODOM_EXPECTED_RATE,
            rate_tol=ODOM_RATE_TOL
        )

        # ===== Subscribers =====
        self.scan_sub = self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.scan_cb,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.imu_cb,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_cb,
            10
        )

        # ===== Timing / shutdown control =====
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        # Main loop timer checks whether the test duration is finished
        self.timer = self.create_timer(0.1, self.loop)

        # Progress timer prints live progress once per second
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info('========================================')
        self.get_logger().info('Starting TurtleBot3 Message Rate Check')
        self.get_logger().info(f'Test duration: {TEST_DURATION:.1f} seconds')
        self.get_logger().info(f'Watching topics: {SCAN_TOPIC}, {IMU_TOPIC}, {ODOM_TOPIC}')
        self.get_logger().info('========================================')

    def scan_cb(self, msg):
        """
        Callback for /scan.
        """
        self.scan_tracker.record_message()

    def imu_cb(self, msg):
        """
        Callback for /imu.
        """
        self.imu_tracker.record_message()

    def odom_cb(self, msg):
        """
        Callback for /odom.
        """
        self.odom_tracker.record_message()

    def progress_update(self):
        """
        Print live progress once per second while the test is running.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        scan_rate = self.scan_tracker.estimated_rate()
        imu_rate = self.imu_tracker.estimated_rate()
        odom_rate = self.odom_tracker.estimated_rate()

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'/scan: {self.scan_tracker.msg_count} msgs ({scan_rate:.2f} Hz) | '
            f'/imu: {self.imu_tracker.msg_count} msgs ({imu_rate:.2f} Hz) | '
            f'/odom: {self.odom_tracker.msg_count} msgs ({odom_rate:.2f} Hz)'
        )

    def log_topic_result(self, test_name, tracker, result):
        """
        Print a clean summary for one topic and append it to the JSON file.
        """
        self.get_logger().info(f'=== {test_name} Results ===')
        self.get_logger().info(f'Topic: {tracker.topic_name}')
        self.get_logger().info(f'Total messages: {result["msg_count"]}')
        self.get_logger().info(f'Elapsed time: {result["elapsed"]:.3f} s')
        self.get_logger().info(f'Average rate: {result["avg_rate"]:.2f} Hz')
        self.get_logger().info(f'Average dt: {result["avg_dt"]:.4f} s')
        self.get_logger().info(f'Min dt: {result["min_dt"]:.4f} s')
        self.get_logger().info(f'Max dt: {result["max_dt"]:.4f} s')
        self.get_logger().info(f'Result: {result["status"]}')

        append_result(
            test_name=test_name,
            status=result['status'],
            measurement=result['measurement'],
            notes=result['notes']
        )

    def finish_and_exit(self):
        """
        Finalize all topic measurements, print results, save JSON results,
        and prepare for shutdown.
        """
        self.get_logger().info('========================================')
        self.get_logger().info('Finalizing message rate results...')
        self.get_logger().info('========================================')

        scan_result = self.scan_tracker.finalize()
        imu_result = self.imu_tracker.finalize()
        odom_result = self.odom_tracker.finalize()

        self.log_topic_result('scan_message_rate', self.scan_tracker, scan_result)
        self.log_topic_result('imu_message_rate', self.imu_tracker, imu_result)
        self.log_topic_result('odom_message_rate', self.odom_tracker, odom_result)

        self.get_logger().info('========================================')
        self.get_logger().info('Message rate check complete.')
        self.get_logger().info('========================================')

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        """
        Main loop:
        - waits until the observation window finishes
        - then computes final results
        - after a short delay, shuts ROS down cleanly
        """
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting message_rate_check')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = MessageRateCheck()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()