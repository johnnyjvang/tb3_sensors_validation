"""
odom_rotation_accuracy.py

Command the robot to rotate in place and measure how accurately odometry
tracks the requested heading change.

Goal:
- Command a fixed in-place rotation
- Measure odom-reported yaw change
- Compare measured yaw change to the target angle
- Save the result to the CSV results file
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
ODOM_TOPIC = '/odom'
CMD_VEL_TOPIC = '/cmd_vel'

TARGET_ROTATION_DEG = 90.0      # target turn angle
ANGULAR_SPEED = 0.4             # rad/s
CONTROL_PERIOD = 0.05           # seconds
SETTLE_TIME = 1.0               # seconds after stop to let odom settle
MAX_TEST_TIME = 10.0            # safety timeout

# Thresholds
PASS_ROT_ERROR_DEG = 5.0
WARN_ROT_ERROR_DEG = 10.0


def quat_to_yaw(x, y, z, w):
    """
    Convert quaternion to yaw in radians.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff_rad(a, b):
    """
    Return shortest signed angle difference a-b in radians.
    """
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


class OdomRotationAccuracy(Node):
    def __init__(self):
        super().__init__('odom_rotation_accuracy')

        # Publisher / subscriber
        self.cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self.sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        # Timing / state
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        # Odom tracking
        self.msg_count = 0
        self.start_yaw = None
        self.current_yaw = None
        self.final_yaw = None

        # Motion state
        self.phase = 'wait_for_odom'
        self.stop_time = None

        # Timers
        self.timer = self.create_timer(CONTROL_PERIOD, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting odom rotation accuracy test on topic: {ODOM_TOPIC}')
        self.get_logger().info(f'Publishing velocity commands on: {CMD_VEL_TOPIC}')
        self.get_logger().info(f'Target rotation: {TARGET_ROTATION_DEG:.1f} deg')
        self.get_logger().info(f'Angular speed: {ANGULAR_SPEED:.2f} rad/s')

    def odom_cb(self, msg):
        """
        Track the current odom yaw.
        """
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.start_yaw is None:
            self.start_yaw = yaw
            self.get_logger().info('First odom message received')

        self.current_yaw = yaw
        self.msg_count += 1

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """
        Publish a TwistStamped command.
        """
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """
        Publish zero velocity to stop the robot.
        """
        self.publish_cmd(0.0, 0.0)

    def progress_update(self):
        """
        Print live progress during the test.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.start_yaw is not None and self.current_yaw is not None:
            turned_rad = angle_diff_rad(self.current_yaw, self.start_yaw)
            turned_deg = math.degrees(turned_rad)
        else:
            turned_deg = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {MAX_TEST_TIME:.1f}s | '
            f'phase: {self.phase} | '
            f'messages: {self.msg_count} | '
            f'turned: {turned_deg:.2f} deg'
        )

    def finish_and_exit(self, status_override=None, notes_override=None):
        """
        Compute final result, save CSV row, and prepare shutdown.
        """
        self.stop_robot()

        if status_override is not None:
            status = status_override
            measurement = '0.00 deg'
            notes = notes_override if notes_override else 'test aborted'
            self.get_logger().error(f'Test failed: {notes}')
        elif self.msg_count < 2 or self.start_yaw is None or self.final_yaw is None:
            status = 'FAIL'
            measurement = '0.00 deg'
            notes = f'No sufficient odom messages received on {ODOM_TOPIC}'
            self.get_logger().error('Test failed: insufficient odom messages received')
        else:
            turned_rad = angle_diff_rad(self.final_yaw, self.start_yaw)
            turned_deg = math.degrees(turned_rad)
            abs_turned_deg = abs(turned_deg)
            rot_error_deg = abs(abs_turned_deg - TARGET_ROTATION_DEG)

            if rot_error_deg <= PASS_ROT_ERROR_DEG:
                status = 'PASS'
            elif rot_error_deg <= WARN_ROT_ERROR_DEG:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{rot_error_deg:.2f} deg'
            notes = (
                f'target={TARGET_ROTATION_DEG:.1f}deg, '
                f'measured={turned_deg:.2f}deg, '
                f'error={rot_error_deg:.2f}deg'
            )

            self.get_logger().info('=== Odom Rotation Accuracy Results ===')
            self.get_logger().info(f'Topic: {ODOM_TOPIC}')
            self.get_logger().info(f'Total messages: {self.msg_count}')
            self.get_logger().info(f'Target rotation: {TARGET_ROTATION_DEG:.2f} deg')
            self.get_logger().info(f'Measured rotation: {turned_deg:.2f} deg')
            self.get_logger().info(f'Rotation error: {rot_error_deg:.2f} deg')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='odom_rotation_accuracy',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        """
        Main control loop for the rotation test.
        """
        now = time.time()
        elapsed = now - self.start_time

        if self.done:
            if now - self.finish_time > 0.5:
                self.get_logger().info('Exiting odom_rotation_accuracy')
                rclpy.shutdown()
            return

        if elapsed > MAX_TEST_TIME:
            self.finish_and_exit(
                status_override='FAIL',
                notes_override='rotation test timed out'
            )
            return

        if self.phase == 'wait_for_odom':
            if self.start_yaw is not None and self.current_yaw is not None:
                self.phase = 'rotating'
                self.get_logger().info('Starting rotation...')
            return

        if self.phase == 'rotating':
            turned_rad = angle_diff_rad(self.current_yaw, self.start_yaw)
            abs_turned_deg = abs(math.degrees(turned_rad))

            if abs_turned_deg < TARGET_ROTATION_DEG:
                self.publish_cmd(0.0, ANGULAR_SPEED)
            else:
                self.stop_robot()
                self.stop_time = now
                self.phase = 'settling'
                self.get_logger().info('Target reached, settling...')
            return

        if self.phase == 'settling':
            self.stop_robot()
            if now - self.stop_time >= SETTLE_TIME:
                self.final_yaw = self.current_yaw
                self.finish_and_exit()
            return


def main(args=None):
    rclpy.init(args=args)
    node = OdomRotationAccuracy()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()