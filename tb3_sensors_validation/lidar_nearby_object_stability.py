"""
lidar_nearby_object_stability.py

Observe the LiDAR scan topic while a nearby object remains fixed in front
of the robot.

Goal:
- Confirm a nearby detected object remains stable over time
- Measure fluctuation and drift in the front-facing range reading
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

TARGET_ANGLE_DEG = 0.0        # front-facing beam
ANGLE_TOL_DEG = 5.0           # acceptable beam search tolerance

# Thresholds
PASS_STABILITY_STD = 0.01     # meters
WARN_STABILITY_STD = 0.03     # meters

PASS_DRIFT = 0.02             # meters
WARN_DRIFT = 0.05             # meters


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


class LidarNearbyObjectStability(Node):
    def __init__(self):
        super().__init__('lidar_nearby_object_stability')

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
        self.valid_sample_count = 0
        self.range_samples = []

        # Timers
        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting LiDAR nearby object stability test on topic: {SCAN_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info('Place a nearby object directly in front of the robot and keep it still.')

    def find_target_index(self, msg):
        """
        Find the scan index closest to TARGET_ANGLE_DEG.
        Returns None if the target angle is outside the scan range or tolerance.
        """
        target_angle_rad = math.radians(TARGET_ANGLE_DEG)
        tol_rad = math.radians(ANGLE_TOL_DEG)

        if msg.angle_increment == 0.0:
            return None

        best_idx = None
        best_error = None

        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            error = abs(angle - target_angle_rad)

            if best_error is None or error < best_error:
                best_error = error
                best_idx = i

        if best_error is None or best_error > tol_rad:
            return None

        return best_idx

    def scan_cb(self, msg):
        """
        Save the front-facing range sample if it is valid.
        """
        idx = self.find_target_index(msg)
        self.msg_count += 1

        if self.msg_count == 1:
            self.get_logger().info('First scan message received')

        if idx is None:
            return

        r = msg.ranges[idx]

        if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
            self.range_samples.append(r)
            self.valid_sample_count += 1

    def progress_update(self):
        """
        Print live progress during the observation window.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if len(self.range_samples) >= 2:
            mean_range = compute_mean(self.range_samples)
            stability_std = compute_std(self.range_samples)
            drift = abs(self.range_samples[-1] - self.range_samples[0])
        else:
            mean_range = 0.0
            stability_std = 0.0
            drift = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'messages: {self.msg_count} | '
            f'valid samples: {self.valid_sample_count} | '
            f'mean range: {mean_range:.4f} m | '
            f'std: {stability_std:.5f} m | '
            f'drift: {drift:.5f} m'
        )

    def finish_and_exit(self):
        """
        Compute final nearby-object stability result and save it.
        """
        if self.msg_count < 1 or len(self.range_samples) < 2:
            status = 'FAIL'
            measurement = '0.00000 m'
            notes = f'No sufficient valid scan samples received on {SCAN_TOPIC}'
            self.get_logger().error('Test failed: insufficient valid scan samples received')
        else:
            mean_range = compute_mean(self.range_samples)
            min_range = min(self.range_samples)
            max_range = max(self.range_samples)
            stability_std = compute_std(self.range_samples)
            drift = abs(self.range_samples[-1] - self.range_samples[0])

            if stability_std <= PASS_STABILITY_STD and drift <= PASS_DRIFT:
                status = 'PASS'
            elif stability_std <= WARN_STABILITY_STD and drift <= WARN_DRIFT:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{stability_std:.5f} m'
            notes = (
                f'mean_range={mean_range:.4f}m, '
                f'min_range={min_range:.4f}m, '
                f'max_range={max_range:.4f}m, '
                f'std={stability_std:.5f}m, '
                f'drift={drift:.5f}m, '
                f'valid_samples={self.valid_sample_count}'
            )

            self.get_logger().info('=== LiDAR Nearby Object Stability Results ===')
            self.get_logger().info(f'Topic: {SCAN_TOPIC}')
            self.get_logger().info(f'Total scan messages: {self.msg_count}')
            self.get_logger().info(f'Valid samples: {self.valid_sample_count}')
            self.get_logger().info(f'Mean range: {mean_range:.4f} m')
            self.get_logger().info(f'Min range: {min_range:.4f} m')
            self.get_logger().info(f'Max range: {max_range:.4f} m')
            self.get_logger().info(f'Stability std: {stability_std:.5f} m')
            self.get_logger().info(f'Drift: {drift:.5f} m')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='lidar_nearby_object_stability',
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
                self.get_logger().info('Exiting lidar_nearby_object_stability')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNearbyObjectStability()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()