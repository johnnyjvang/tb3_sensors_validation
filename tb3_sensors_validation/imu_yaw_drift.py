"""
imu_yaw_drift.py

Observe the IMU topic while the robot remains stationary.

Goal:
- Confirm IMU yaw remains stable while the robot is not moving
- Measure total yaw drift over a fixed time window
- Save the result to the CSV results file
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
IMU_TOPIC = '/imu'
TEST_DURATION = 10.0          # seconds to observe IMU while stationary

# Thresholds
PASS_YAW_DRIFT_DEG = 2.0
WARN_YAW_DRIFT_DEG = 5.0


def quat_to_yaw(x, y, z, w):
    """
    Convert a quaternion to yaw in radians.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff_rad(a, b):
    """
    Return the shortest signed difference between two angles in radians.
    """
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


class ImuYawDrift(Node):
    def __init__(self):
        super().__init__('imu_yaw_drift')

        # Subscriber
        self.sub = self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.imu_cb,
            10
        )

        # Timing / test control
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        # Yaw tracking
        self.msg_count = 0
        self.first_yaw = None
        self.last_yaw = None

        # Timers
        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting IMU yaw drift test on topic: {IMU_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info('Keep the robot completely still during this test.')

    def imu_cb(self, msg):
        """
        Save the first and most recent IMU yaw.
        """
        q = msg.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.first_yaw is None:
            self.first_yaw = yaw
            self.get_logger().info('First IMU message received')

        self.last_yaw = yaw
        self.msg_count += 1

    def progress_update(self):
        """
        Print live progress during the observation window.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.first_yaw is not None and self.last_yaw is not None:
            yaw_drift_rad = angle_diff_rad(self.last_yaw, self.first_yaw)
            yaw_drift_deg = abs(math.degrees(yaw_drift_rad))
        else:
            yaw_drift_deg = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'messages: {self.msg_count} | '
            f'yaw drift: {yaw_drift_deg:.2f} deg'
        )

    def finish_and_exit(self):
        """
        Compute final yaw drift result and save it.
        """
        if self.msg_count < 2 or self.first_yaw is None or self.last_yaw is None:
            status = 'FAIL'
            measurement = '0.00 deg'
            notes = f'No sufficient IMU messages received on {IMU_TOPIC}'
            self.get_logger().error('Test failed: insufficient IMU messages received')
        else:
            yaw_drift_rad = angle_diff_rad(self.last_yaw, self.first_yaw)
            yaw_drift_deg = abs(math.degrees(yaw_drift_rad))

            if yaw_drift_deg <= PASS_YAW_DRIFT_DEG:
                status = 'PASS'
            elif yaw_drift_deg <= WARN_YAW_DRIFT_DEG:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{yaw_drift_deg:.2f} deg'
            notes = f'yaw_drift={yaw_drift_deg:.2f}deg over {TEST_DURATION:.1f}s'

            self.get_logger().info('=== IMU Yaw Drift Results ===')
            self.get_logger().info(f'Topic: {IMU_TOPIC}')
            self.get_logger().info(f'Total messages: {self.msg_count}')
            self.get_logger().info(f'Yaw drift: {yaw_drift_deg:.2f} deg')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='imu_yaw_drift',
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
                self.get_logger().info('Exiting imu_yaw_drift')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawDrift()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()