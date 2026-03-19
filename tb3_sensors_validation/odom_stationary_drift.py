"""
odom_stationary_drift.py

Observe the odometry topic while the robot remains stationary.

Goal:
- Confirm odom is stable while the robot is not moving
- Measure drift in x, y, and yaw over a fixed time window
- Save the result to the CSV results file
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
ODOM_TOPIC = '/odom'
TEST_DURATION = 10.0          # seconds to observe odom while stationary

# Position drift thresholds
PASS_POS_DRIFT = 0.02         # meters
WARN_POS_DRIFT = 0.05         # meters

# Yaw drift thresholds
PASS_YAW_DRIFT_DEG = 2.0      # degrees
WARN_YAW_DRIFT_DEG = 5.0      # degrees


def quat_to_yaw(x, y, z, w):
    """
    Convert a quaternion to yaw in radians.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff_rad(a, b):
    """
    Compute the shortest signed angular difference between two angles.
    """
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


class OdomStationaryDrift(Node):
    def __init__(self):
        super().__init__('odom_stationary_drift')

        # Subscriber
        self.sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_cb,
            10
        )

        # Timing / test control
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        # Odom state tracking
        self.msg_count = 0
        self.first_pose = None
        self.last_pose = None

        # Timers
        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting odom stationary drift test on topic: {ODOM_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info('Keep the robot completely still during this test.')

    def odom_cb(self, msg):
        """
        Save the first and latest odom pose.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        pose = {
            'x': x,
            'y': y,
            'yaw': yaw,
        }

        if self.first_pose is None:
            self.first_pose = pose
            self.get_logger().info('First odom message received')

        self.last_pose = pose
        self.msg_count += 1

    def progress_update(self):
        """
        Print live drift progress during the observation window.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.first_pose is not None and self.last_pose is not None:
            dx = self.last_pose['x'] - self.first_pose['x']
            dy = self.last_pose['y'] - self.first_pose['y']
            pos_drift = math.sqrt(dx * dx + dy * dy)

            yaw_drift_rad = angle_diff_rad(self.last_pose['yaw'], self.first_pose['yaw'])
            yaw_drift_deg = abs(math.degrees(yaw_drift_rad))
        else:
            pos_drift = 0.0
            yaw_drift_deg = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'messages: {self.msg_count} | '
            f'pos drift: {pos_drift:.4f} m | '
            f'yaw drift: {yaw_drift_deg:.2f} deg'
        )

    def finish_and_exit(self):
        """
        Compute final odom drift metrics and save the result.
        """
        if self.msg_count < 2 or self.first_pose is None or self.last_pose is None:
            status = 'FAIL'
            measurement = '0.0000 m'
            notes = f'No sufficient odom messages received on {ODOM_TOPIC}'
            self.get_logger().error('Test failed: insufficient odom messages received')
        else:
            dx = self.last_pose['x'] - self.first_pose['x']
            dy = self.last_pose['y'] - self.first_pose['y']
            pos_drift = math.sqrt(dx * dx + dy * dy)

            yaw_drift_rad = angle_diff_rad(self.last_pose['yaw'], self.first_pose['yaw'])
            yaw_drift_deg = abs(math.degrees(yaw_drift_rad))

            if pos_drift <= PASS_POS_DRIFT and yaw_drift_deg <= PASS_YAW_DRIFT_DEG:
                status = 'PASS'
            elif pos_drift <= WARN_POS_DRIFT and yaw_drift_deg <= WARN_YAW_DRIFT_DEG:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{pos_drift:.4f} m'
            notes = (
                f'x_drift={dx:.4f}m, '
                f'y_drift={dy:.4f}m, '
                f'pos_drift={pos_drift:.4f}m, '
                f'yaw_drift={yaw_drift_deg:.2f}deg'
            )

            self.get_logger().info('=== Odom Stationary Drift Results ===')
            self.get_logger().info(f'Topic: {ODOM_TOPIC}')
            self.get_logger().info(f'Total messages: {self.msg_count}')
            self.get_logger().info(f'X drift: {dx:.4f} m')
            self.get_logger().info(f'Y drift: {dy:.4f} m')
            self.get_logger().info(f'Position drift: {pos_drift:.4f} m')
            self.get_logger().info(f'Yaw drift: {yaw_drift_deg:.2f} deg')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='odom_stationary_drift',
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
                self.get_logger().info('Exiting odom_stationary_drift')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = OdomStationaryDrift()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()