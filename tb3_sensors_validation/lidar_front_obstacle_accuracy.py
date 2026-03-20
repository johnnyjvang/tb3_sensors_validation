"""
lidar_front_obstacle_accuracy.py

Detect whether there is an obstacle in front of the robot using LaserScan data.

Goal:
- Confirm the LiDAR can detect something in front of the robot
- Report nearest front obstacle distance
- Report average front obstacle distance
- Estimate the angular width of the detected front obstacle region
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
TEST_DURATION = 10.0

FRONT_WINDOW_DEG = 15.0          # analyze only +/- this many degrees around center
MAX_DETECTION_RANGE = 1.5        # consider only obstacles closer than this distance
MIN_VALID_HITS = 3               # require at least this many valid front hits
MIN_CLUSTER_BEAMS = 3            # require this many consecutive beams to count as a front obstacle


class LidarFrontObstacleAccuracy(Node):
    def __init__(self):
        super().__init__('lidar_front_obstacle_accuracy')

        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, 10)

        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        self.scan_count = 0

        self.front_min_measurements = []
        self.front_avg_measurements = []
        self.cluster_width_measurements = []
        self.detected_scan_count = 0

        self.last_front_min = None
        self.last_front_avg = None
        self.last_cluster_width_deg = None
        self.last_hit_count = 0

        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting front obstacle detection test on topic: {SCAN_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info(
            f'Checking front sector +/- {FRONT_WINDOW_DEG:.1f} deg within {MAX_DETECTION_RANGE:.2f} m'
        )

    def front_points(self, msg):
        """
        Return valid front-sector scan points within the detection range.
        """
        points = []

        min_rad = math.radians(-FRONT_WINDOW_DEG)
        max_rad = math.radians(FRONT_WINDOW_DEG)

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if angle < min_rad or angle > max_rad:
                continue

            if math.isnan(r) or math.isinf(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            if r > MAX_DETECTION_RANGE:
                continue

            points.append((i, angle, r))

        return points

    def largest_consecutive_cluster(self, points):
        """
        Find the largest consecutive beam cluster from the valid front points.
        """
        if not points:
            return []

        clusters = []
        current_cluster = [points[0]]

        for prev, curr in zip(points[:-1], points[1:]):
            prev_i = prev[0]
            curr_i = curr[0]

            if curr_i == prev_i + 1:
                current_cluster.append(curr)
            else:
                clusters.append(current_cluster)
                current_cluster = [curr]

        clusters.append(current_cluster)

        largest = max(clusters, key=len)
        return largest

    def scan_cb(self, msg):
        """
        Process one LaserScan message and record front obstacle measurements if detected.
        """
        self.scan_count += 1

        points = self.front_points(msg)
        self.last_hit_count = len(points)

        if len(points) < MIN_VALID_HITS:
            return

        cluster = self.largest_consecutive_cluster(points)

        if len(cluster) < MIN_CLUSTER_BEAMS:
            return

        distances = [p[2] for p in cluster]
        angles_deg = [math.degrees(p[1]) for p in cluster]

        front_min = min(distances)
        front_avg = sum(distances) / len(distances)
        cluster_width_deg = max(angles_deg) - min(angles_deg)

        self.last_front_min = front_min
        self.last_front_avg = front_avg
        self.last_cluster_width_deg = cluster_width_deg

        self.front_min_measurements.append(front_min)
        self.front_avg_measurements.append(front_avg)
        self.cluster_width_measurements.append(cluster_width_deg)
        self.detected_scan_count += 1

    def compute_stddev(self, samples):
        """
        Compute population standard deviation.
        """
        if len(samples) < 2:
            return 0.0
        mean_val = sum(samples) / len(samples)
        variance = sum((x - mean_val) ** 2 for x in samples) / len(samples)
        return math.sqrt(variance)

    def progress_update(self):
        """
        Print live progress during the observation window.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.front_min_measurements:
            avg_front_min = sum(self.front_min_measurements) / len(self.front_min_measurements)
            avg_front_avg = sum(self.front_avg_measurements) / len(self.front_avg_measurements)
            avg_cluster_width = sum(self.cluster_width_measurements) / len(self.cluster_width_measurements)

            self.get_logger().info(
                f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                f'scans: {self.scan_count} | '
                f'detected scans: {self.detected_scan_count} | '
                f'nearest front: {avg_front_min:.3f} m | '
                f'front avg: {avg_front_avg:.3f} m | '
                f'cluster width: {avg_cluster_width:.1f} deg'
            )
        else:
            self.get_logger().info(
                f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                f'scans: {self.scan_count} | '
                f'front hits: {self.last_hit_count} | '
                f'waiting for detectable front obstacle'
            )

    def finish_and_exit(self):
        """
        Finalize test result and save it to CSV.
        """
        if not self.front_min_measurements:
            status = 'FAIL'
            measurement = 'No obstacle detected'
            notes = 'No valid front obstacle cluster detected'
            self.get_logger().error('Test failed: no valid front obstacle detected')
        else:
            avg_front_min = sum(self.front_min_measurements) / len(self.front_min_measurements)
            avg_front_avg = sum(self.front_avg_measurements) / len(self.front_avg_measurements)
            avg_cluster_width = sum(self.cluster_width_measurements) / len(self.cluster_width_measurements)

            front_min_std = self.compute_stddev(self.front_min_measurements)
            front_avg_std = self.compute_stddev(self.front_avg_measurements)
            cluster_width_std = self.compute_stddev(self.cluster_width_measurements)

            detection_ratio = self.detected_scan_count / self.scan_count if self.scan_count > 0 else 0.0

            status = 'PASS'
            measurement = f'{avg_front_min:.3f} m'
            notes = (
                f'front_avg={avg_front_avg:.3f}m, '
                f'width={avg_cluster_width:.1f}deg, '
                f'detection_ratio={detection_ratio:.2f}'
            )

            self.get_logger().info('=== Front Obstacle Detection Results ===')
            self.get_logger().info(f'Total scans received: {self.scan_count}')
            self.get_logger().info(f'Scans with valid front obstacle: {self.detected_scan_count}')
            self.get_logger().info(f'Detection ratio: {detection_ratio:.2f}')
            self.get_logger().info(f'Average nearest front distance: {avg_front_min:.3f} m')
            self.get_logger().info(f'Nearest front distance std dev: {front_min_std:.4f} m')
            self.get_logger().info(f'Average front distance: {avg_front_avg:.3f} m')
            self.get_logger().info(f'Front average std dev: {front_avg_std:.4f} m')
            self.get_logger().info(f'Average front cluster width: {avg_cluster_width:.1f} deg')
            self.get_logger().info(f'Front cluster width std dev: {cluster_width_std:.2f} deg')
            self.get_logger().info('Result: PASS')

        append_result(
            test_name='lidar_front_obstacle_accuracy',
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
                self.get_logger().info('Exiting lidar_front_obstacle_accuracy')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = LidarFrontObstacleAccuracy()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()