"""
odom_forward_accuracy.py

Command the robot to drive forward and measure odometry distance accuracy.

Goal:
- Command a forward motion using /cmd_vel
- Measure odom-reported distance traveled
- Compare measured distance to target distance
- Save the result to the CSV results file
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
CMD_VEL_TOPIC = '/cmd_vel'
ODOM_TOPIC = '/odom'

TARGET_DISTANCE = 1.0         # meters
LINEAR_SPEED = 0.10           # m/s
TEST_TIMEOUT = 20.0           # seconds

PASS_ERROR = 0.03             # meters
WARN_ERROR = 0.07             # meters


class OdomForwardAccuracy(Node):
    def __init__(self):
        super().__init__('odom_forward_accuracy')

        # Publisher / subscriber
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        # Test state
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        self.first_pose = None
        self.last_pose = None
        self.msg_count = 0
        self.motion_started = False

        # Timers
        self.timer = self.create_timer(0.05, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info('Starting odom forward accuracy test')
        self.get_logger().info(f'Target distance: {TARGET_DISTANCE:.3f} m')
        self.get_logger().info(f'Linear speed: {LINEAR_SPEED:.3f} m/s')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        pose = {'x': x, 'y': y}

        if self.first_pose is None:
            self.first_pose = pose
            self.get_logger().info('First odom message received')

        self.last_pose = pose
        self.msg_count += 1

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        for _ in range(5):
            self.publish_cmd(0.0, 0.0)

    def get_distance_traveled(self):
        if self.first_pose is None or self.last_pose is None:
            return 0.0

        dx = self.last_pose['x'] - self.first_pose['x']
        dy = self.last_pose['y'] - self.first_pose['y']
        return math.sqrt(dx * dx + dy * dy)

    def progress_update(self):
        if self.done:
            return

        elapsed = time.time() - self.start_time
        dist = self.get_distance_traveled()

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_TIMEOUT:.1f}s | '
            f'messages: {self.msg_count} | '
            f'distance: {dist:.4f} m / {TARGET_DISTANCE:.4f} m'
        )

    def finish_and_exit(self):
        self.stop_robot()

        if self.msg_count < 2 or self.first_pose is None or self.last_pose is None:
            status = 'FAIL'
            measurement = '0.0000 m'
            notes = f'No sufficient odom messages received on {ODOM_TOPIC}'
            self.get_logger().error('Test failed: insufficient odom messages received')
        else:
            dist = self.get_distance_traveled()
            error = abs(dist - TARGET_DISTANCE)

            if error <= PASS_ERROR:
                status = 'PASS'
            elif error <= WARN_ERROR:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{dist:.4f} m'
            notes = (
                f'target={TARGET_DISTANCE:.4f}m, '
                f'measured={dist:.4f}m, '
                f'error={error:.4f}m'
            )

            self.get_logger().info('=== Odom Forward Accuracy Results ===')
            self.get_logger().info(f'Target distance: {TARGET_DISTANCE:.4f} m')
            self.get_logger().info(f'Measured distance: {dist:.4f} m')
            self.get_logger().info(f'Absolute error: {error:.4f} m')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='odom_forward_accuracy',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting odom_forward_accuracy')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time

        if self.first_pose is not None:
            self.motion_started = True
            dist = self.get_distance_traveled()

            if dist < TARGET_DISTANCE:
                self.publish_cmd(LINEAR_SPEED, 0.0)
            else:
                self.finish_and_exit()
                return

        if elapsed >= TEST_TIMEOUT:
            self.get_logger().warn('Test timed out')
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = OdomForwardAccuracy()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()