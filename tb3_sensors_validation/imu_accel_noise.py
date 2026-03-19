"""
imu_accel_noise.py

Observe the IMU topic while the robot remains stationary.

Goal:
- Confirm IMU linear acceleration is stable while the robot is not moving
- Measure accelerometer noise on x, y, and z axes
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

# Thresholds based mainly on x/y accel noise (m/s^2)
PASS_ACCEL_NOISE_XY = 0.05
WARN_ACCEL_NOISE_XY = 0.15


def compute_mean(values):
    """
    Return the mean of a list of numeric values.
    """
    if not values:
        return 0.0
    return sum(values) / len(values)


def compute_std(values):
    """
    Return the population standard deviation of a list of numeric values.
    """
    if len(values) < 2:
        return 0.0

    mean_val = compute_mean(values)
    variance = sum((x - mean_val) ** 2 for x in values) / len(values)
    return math.sqrt(variance)


class ImuAccelNoise(Node):
    def __init__(self):
        super().__init__('imu_accel_noise')

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

        # Data storage
        self.msg_count = 0
        self.accel_x_list = []
        self.accel_y_list = []
        self.accel_z_list = []

        # Timers
        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting IMU accel noise test on topic: {IMU_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info('Keep the robot completely still during this test.')

    def imu_cb(self, msg):
        """
        Save linear acceleration samples from the IMU.
        """
        self.accel_x_list.append(msg.linear_acceleration.x)
        self.accel_y_list.append(msg.linear_acceleration.y)
        self.accel_z_list.append(msg.linear_acceleration.z)
        self.msg_count += 1

        if self.msg_count == 1:
            self.get_logger().info('First IMU message received')

    def progress_update(self):
        """
        Print live progress during the observation window.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        noise_x = compute_std(self.accel_x_list)
        noise_y = compute_std(self.accel_y_list)
        noise_z = compute_std(self.accel_z_list)

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'messages: {self.msg_count} | '
            f'accel noise x: {noise_x:.5f} | '
            f'y: {noise_y:.5f} | '
            f'z: {noise_z:.5f} m/s^2'
        )

    def finish_and_exit(self):
        """
        Compute final accelerometer noise metrics and save the result.
        """
        if self.msg_count < 2:
            status = 'FAIL'
            measurement = '0.00000 m/s^2'
            notes = f'No sufficient IMU messages received on {IMU_TOPIC}'
            self.get_logger().error('Test failed: insufficient IMU messages received')
        else:
            mean_x = compute_mean(self.accel_x_list)
            mean_y = compute_mean(self.accel_y_list)
            mean_z = compute_mean(self.accel_z_list)

            noise_x = compute_std(self.accel_x_list)
            noise_y = compute_std(self.accel_y_list)
            noise_z = compute_std(self.accel_z_list)

            noise_xy = max(noise_x, noise_y)

            if noise_xy <= PASS_ACCEL_NOISE_XY:
                status = 'PASS'
            elif noise_xy <= WARN_ACCEL_NOISE_XY:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{noise_xy:.5f} m/s^2'
            notes = (
                f'accel_mean_x={mean_x:.5f}m/s^2, '
                f'accel_mean_y={mean_y:.5f}m/s^2, '
                f'accel_mean_z={mean_z:.5f}m/s^2, '
                f'accel_noise_x={noise_x:.5f}m/s^2, '
                f'accel_noise_y={noise_y:.5f}m/s^2, '
                f'accel_noise_z={noise_z:.5f}m/s^2'
            )

            self.get_logger().info('=== IMU Accel Noise Results ===')
            self.get_logger().info(f'Topic: {IMU_TOPIC}')
            self.get_logger().info(f'Total messages: {self.msg_count}')
            self.get_logger().info(f'Accel mean x: {mean_x:.5f} m/s^2')
            self.get_logger().info(f'Accel mean y: {mean_y:.5f} m/s^2')
            self.get_logger().info(f'Accel mean z: {mean_z:.5f} m/s^2')
            self.get_logger().info(f'Accel noise x: {noise_x:.5f} m/s^2')
            self.get_logger().info(f'Accel noise y: {noise_y:.5f} m/s^2')
            self.get_logger().info(f'Accel noise z: {noise_z:.5f} m/s^2')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='imu_accel_noise',
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
                self.get_logger().info('Exiting imu_accel_noise')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = ImuAccelNoise()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()