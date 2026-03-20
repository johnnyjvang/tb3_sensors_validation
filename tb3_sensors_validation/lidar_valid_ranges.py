"""
lidar_valid_ranges.py

Observe the LiDAR scan topic and measure how many reported ranges are valid.

Goal:
- Confirm the LiDAR topic is publishing usable data
- Measure the percentage of valid scan ranges
- Save the result to the CSV results file
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
SCAN_TOPIC = '/scan'
TEST_DURATION = 10.0          # seconds to observe LiDAR messages

# Thresholds
PASS_VALID_PERCENT = 95.0
WARN_VALID_PERCENT = 85.0


class LidarValidRanges(Node):
    def __init__(self):
        super().__init__('lidar_valid_ranges')

        # Subscriber
        self.sub = self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.scan_cb,
            10
        )

        # Timing / test control
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        # Data tracking
        self.msg_count = 0
        self.total_points = 0
        self.valid_points = 0
        self.invalid_points = 0

        # Timers
        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting LiDAR valid ranges test on topic: {SCAN_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')

    def scan_cb(self, msg):
        """
        Count valid and invalid range values in each LaserScan message.
        """
        valid_in_msg = 0
        invalid_in_msg = 0

        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                valid_in_msg += 1
            else:
                invalid_in_msg += 1

        self.valid_points += valid_in_msg
        self.invalid_points += invalid_in_msg
        self.total_points += valid_in_msg + invalid_in_msg
        self.msg_count += 1

        if self.msg_count == 1:
            self.get_logger().info('First scan message received')

    def progress_update(self):
        """
        Print live progress during the observation window.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.total_points > 0:
            valid_percent = 100.0 * self.valid_points / self.total_points
        else:
            valid_percent = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'messages: {self.msg_count} | '
            f'valid points: {self.valid_points} / {self.total_points} '
            f'({valid_percent:.2f}%)'
        )

    def finish_and_exit(self):
        """
        Compute final valid-range result and save it.
        """
        if self.msg_count < 1 or self.total_points == 0:
            status = 'FAIL'
            measurement = '0.00 %'
            notes = f'No sufficient scan messages received on {SCAN_TOPIC}'
            self.get_logger().error('Test failed: insufficient scan messages received')
        else:
            valid_percent = 100.0 * self.valid_points / self.total_points

            if valid_percent >= PASS_VALID_PERCENT:
                status = 'PASS'
            elif valid_percent >= WARN_VALID_PERCENT:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{valid_percent:.2f} %'
            notes = (
                f'valid_points={self.valid_points}, '
                f'invalid_points={self.invalid_points}, '
                f'total_points={self.total_points}, '
                f'valid_percent={valid_percent:.2f}%'
            )

            self.get_logger().info('=== LiDAR Valid Ranges Results ===')
            self.get_logger().info(f'Topic: {SCAN_TOPIC}')
            self.get_logger().info(f'Total scan messages: {self.msg_count}')
            self.get_logger().info(f'Valid points: {self.valid_points}')
            self.get_logger().info(f'Invalid points: {self.invalid_points}')
            self.get_logger().info(f'Total points: {self.total_points}')
            self.get_logger().info(f'Valid percentage: {valid_percent:.2f} %')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='lidar_valid_ranges',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        """
        End the test after the observation window, then shut down cleanly.
        """
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting lidar_valid_ranges')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = LidarValidRanges()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()