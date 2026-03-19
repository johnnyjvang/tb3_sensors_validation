"""
imu_gyro_bias.py

Observe the IMU topic while the robot remains stationary.

Goal:
- Confirm IMU angular velocity is near zero while the robot is not moving
- Measure average gyro bias on x, y, and z axes
- Save the result to the CSV results file
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
IMU_TOPIC = '/imu'
TEST_DURATION = 10.0          # seconds to observe IMU while stationary

# Thresholds based mainly on z-axis gyro bias (rad/s)
PASS_GYRO_BIAS_Z = 0.02
WARN_GYRO_BIAS_Z = 0.05


class ImuGyroBias(Node):
    def __init__(self):
        super().__init__('imu_gyro_bias')

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
        self.gyro_x_list = []
        self.gyro_y_list = []
        self.gyro_z_list = []

        # Timers
        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting IMU gyro bias test on topic: {IMU_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info('Keep the robot completely still during this test.')

    def imu_cb(self, msg):
        """
        Save angular velocity samples from the IMU.
        """
        self.gyro_x_list.append(msg.angular_velocity.x)
        self.gyro_y_list.append(msg.angular_velocity.y)
        self.gyro_z_list.append(msg.angular_velocity.z)
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

        if self.msg_count > 0:
            mean_x = sum(self.gyro_x_list) / len(self.gyro_x_list)
            mean_y = sum(self.gyro_y_list) / len(self.gyro_y_list)
            mean_z = sum(self.gyro_z_list) / len(self.gyro_z_list)
        else:
            mean_x = 0.0
            mean_y = 0.0
            mean_z = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'messages: {self.msg_count} | '
            f'gyro bias x: {mean_x:.5f} | '
            f'y: {mean_y:.5f} | '
            f'z: {mean_z:.5f} rad/s'
        )

    def finish_and_exit(self):
        """
        Compute final gyro bias metrics and save the result.
        """
        if self.msg_count < 2:
            status = 'FAIL'
            measurement = '0.00000 rad/s'
            notes = f'No sufficient IMU messages received on {IMU_TOPIC}'
            self.get_logger().error('Test failed: insufficient IMU messages received')
        else:
            mean_x = sum(self.gyro_x_list) / len(self.gyro_x_list)
            mean_y = sum(self.gyro_y_list) / len(self.gyro_y_list)
            mean_z = sum(self.gyro_z_list) / len(self.gyro_z_list)

            abs_mean_z = abs(mean_z)

            if abs_mean_z <= PASS_GYRO_BIAS_Z:
                status = 'PASS'
            elif abs_mean_z <= WARN_GYRO_BIAS_Z:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{abs_mean_z:.5f} rad/s'
            notes = (
                f'gyro_bias_x={mean_x:.5f}rad/s, '
                f'gyro_bias_y={mean_y:.5f}rad/s, '
                f'gyro_bias_z={mean_z:.5f}rad/s'
            )

            self.get_logger().info('=== IMU Gyro Bias Results ===')
            self.get_logger().info(f'Topic: {IMU_TOPIC}')
            self.get_logger().info(f'Total messages: {self.msg_count}')
            self.get_logger().info(f'Gyro bias x: {mean_x:.5f} rad/s')
            self.get_logger().info(f'Gyro bias y: {mean_y:.5f} rad/s')
            self.get_logger().info(f'Gyro bias z: {mean_z:.5f} rad/s')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='imu_gyro_bias',
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
                self.get_logger().info('Exiting imu_gyro_bias')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = ImuGyroBias()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()