"""
odom_backward_accuracy.py

Command the robot to drive backward and measure how accurately odometry
tracks the requested travel distance.

Goal:
- Command a fixed backward motion
- Measure odom-reported distance traveled
- Compare measured distance to the target distance
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

TARGET_DISTANCE = 1.0          # meters
LINEAR_SPEED = -0.10           # m/s (negative for backward)
CONTROL_PERIOD = 0.05          # seconds
SETTLE_TIME = 1.0              # seconds after stop to let odom settle
MAX_TEST_TIME = 20.0           # safety timeout

# Thresholds
PASS_DISTANCE_ERROR = 0.03     # meters
WARN_DISTANCE_ERROR = 0.07     # meters


def planar_distance(x1, y1, x2, y2):
    """
    Euclidean distance in the XY plane.
    """
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


class OdomBackwardAccuracy(Node):
    def __init__(self):
        super().__init__('odom_backward_accuracy')

        # Publisher / subscriber
        self.cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self.sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        # Timing / state
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        # Odom tracking
        self.msg_count = 0
        self.start_pose = None
        self.current_pose = None
        self.final_pose = None

        # Motion state
        self.phase = 'wait_for_odom'
        self.stop_time = None

        # Timers
        self.timer = self.create_timer(CONTROL_PERIOD, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting odom backward accuracy test on topic: {ODOM_TOPIC}')
        self.get_logger().info(f'Publishing velocity commands on: {CMD_VEL_TOPIC}')
        self.get_logger().info(f'Target distance: {TARGET_DISTANCE:.2f} m')
        self.get_logger().info(f'Linear speed: {LINEAR_SPEED:.2f} m/s')

    def odom_cb(self, msg):
        """
        Track current odom pose.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        pose = {
            'x': x,
            'y': y,
        }

        if self.start_pose is None:
            self.start_pose = pose
            self.get_logger().info('First odom message received')

        self.current_pose = pose
        self.msg_count += 1

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """
        Publish a TwistStamped command.
        """
        cmd = TwistStamped()
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

        if self.start_pose is not None and self.current_pose is not None:
            dist = planar_distance(
                self.start_pose['x'], self.start_pose['y'],
                self.current_pose['x'], self.current_pose['y']
            )
        else:
            dist = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {MAX_TEST_TIME:.1f}s | '
            f'phase: {self.phase} | '
            f'messages: {self.msg_count} | '
            f'distance: {dist:.3f} m'
        )

    def finish_and_exit(self, status_override=None, notes_override=None):
        """
        Compute final result, save CSV row, and prepare shutdown.
        """
        self.stop_robot()

        if status_override is not None:
            status = status_override
            measurement = '0.0000 m'
            notes = notes_override if notes_override else 'test aborted'
            self.get_logger().error(f'Test failed: {notes}')
        elif self.msg_count < 2 or self.start_pose is None or self.final_pose is None:
            status = 'FAIL'
            measurement = '0.0000 m'
            notes = f'No sufficient odom messages received on {ODOM_TOPIC}'
            self.get_logger().error('Test failed: insufficient odom messages received')
        else:
            measured_distance = planar_distance(
                self.start_pose['x'], self.start_pose['y'],
                self.final_pose['x'], self.final_pose['y']
            )
            distance_error = abs(measured_distance - TARGET_DISTANCE)

            if distance_error <= PASS_DISTANCE_ERROR:
                status = 'PASS'
            elif distance_error <= WARN_DISTANCE_ERROR:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{distance_error:.4f} m'
            notes = (
                f'target={TARGET_DISTANCE:.4f}m, '
                f'measured={measured_distance:.4f}m, '
                f'error={distance_error:.4f}m'
            )

            self.get_logger().info('=== Odom Backward Accuracy Results ===')
            self.get_logger().info(f'Topic: {ODOM_TOPIC}')
            self.get_logger().info(f'Total messages: {self.msg_count}')
            self.get_logger().info(f'Target distance: {TARGET_DISTANCE:.4f} m')
            self.get_logger().info(f'Measured distance: {measured_distance:.4f} m')
            self.get_logger().info(f'Distance error: {distance_error:.4f} m')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='odom_backward_accuracy',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        """
        Main control loop for the backward accuracy test.
        """
        now = time.time()
        elapsed = now - self.start_time

        if self.done:
            if now - self.finish_time > 0.5:
                self.get_logger().info('Exiting odom_backward_accuracy')
                rclpy.shutdown()
            return

        if elapsed > MAX_TEST_TIME:
            self.finish_and_exit(
                status_override='FAIL',
                notes_override='backward accuracy test timed out'
            )
            return

        if self.phase == 'wait_for_odom':
            if self.start_pose is not None and self.current_pose is not None:
                self.phase = 'moving'
                self.get_logger().info('Starting backward motion...')
            return

        if self.phase == 'moving':
            dist = planar_distance(
                self.start_pose['x'], self.start_pose['y'],
                self.current_pose['x'], self.current_pose['y']
            )

            if dist < TARGET_DISTANCE:
                self.publish_cmd(LINEAR_SPEED, 0.0)
            else:
                self.stop_robot()
                self.stop_time = now
                self.phase = 'settling'
                self.get_logger().info('Target distance reached, settling...')
            return

        if self.phase == 'settling':
            self.stop_robot()
            if now - self.stop_time >= SETTLE_TIME:
                self.final_pose = dict(self.current_pose)
                self.finish_and_exit()
            return


def main(args=None):
    rclpy.init(args=args)
    node = OdomBackwardAccuracy()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()