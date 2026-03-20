"""
odom_out_and_back.py

Command the robot to drive forward, rotate 180 degrees, and drive back.

Goal:
- Move forward a fixed distance
- Rotate 180 degrees
- Move back the same distance
- Measure final odom closure error relative to the starting pose
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

TARGET_DISTANCE = 1.0
TARGET_ROTATION_DEG = 180.0
ROTATION_STOP_TOLERANCE_DEG = 0.5

LINEAR_SPEED = 0.10
ANGULAR_SPEED_FAST = 0.30
ANGULAR_SPEED_SLOW = 0.10

CONTROL_PERIOD = 0.05
SETTLE_TIME = 1.0
MAX_TEST_TIME = 40.0

# Thresholds
PASS_CLOSURE_ERROR = 0.08
WARN_CLOSURE_ERROR = 0.15

PASS_FINAL_YAW_ERROR_DEG = 8.0
WARN_FINAL_YAW_ERROR_DEG = 15.0


def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion to yaw in radians.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def planar_distance(x1, y1, x2, y2):
    """
    Euclidean distance in the XY plane.
    """
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


class OdomOutAndBack(Node):
    def __init__(self):
        super().__init__('odom_out_and_back')

        self.pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        # Start pose
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        # Current pose
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # Per-phase start pose
        self.phase_start_x = None
        self.phase_start_y = None
        self.phase_start_yaw = None

        # Measurements
        self.forward_distance = 0.0
        self.rotation_deg = 0.0
        self.return_distance = 0.0

        # State
        self.phase = 'waiting_for_data'
        self.phase_start_time = time.time()
        self.last_progress_log_time = 0.0
        self.wait_log_time = 0.0

        self.done = False
        self.finish_time = None

        self.timer = self.create_timer(CONTROL_PERIOD, self.loop)

        self.get_logger().info('Starting odom_out_and_back test')
        self.get_logger().info(f'Target distance: {TARGET_DISTANCE:.2f} m each leg')
        self.get_logger().info(f'Target rotation: {TARGET_ROTATION_DEG:.1f} deg')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def data_ready(self):
        return (
            self.current_x is not None and
            self.current_y is not None and
            self.current_yaw is not None
        )

    def publish(self, x=0.0, z=0.0):
        msg = TwistStamped()
        msg.twist.linear.x = x
        msg.twist.angular.z = z
        self.pub.publish(msg)

    def set_new_phase(self, new_phase):
        self.phase = new_phase
        self.phase_start_time = time.time()
        self.phase_start_x = self.current_x
        self.phase_start_y = self.current_y
        self.phase_start_yaw = self.current_yaw
        self.last_progress_log_time = 0.0
        self.get_logger().info(f'Starting phase: {new_phase}')

    def maybe_log_progress(self, message):
        now = time.time()
        if now - self.last_progress_log_time > 0.5:
            self.get_logger().info(message)
            self.last_progress_log_time = now

    def finish_and_exit(self):
        self.publish()
        time.sleep(0.1)
        self.publish()

        closure_error = math.sqrt(
            (self.current_x - self.start_x) ** 2 +
            (self.current_y - self.start_y) ** 2
        )

        final_yaw_error_deg = abs(
            math.degrees(normalize_angle(self.current_yaw - self.start_yaw))
        )

        if closure_error <= PASS_CLOSURE_ERROR and final_yaw_error_deg <= PASS_FINAL_YAW_ERROR_DEG:
            status = 'PASS'
        elif closure_error <= WARN_CLOSURE_ERROR and final_yaw_error_deg <= WARN_FINAL_YAW_ERROR_DEG:
            status = 'WARN'
        else:
            status = 'FAIL'

        measurement = f'{closure_error:.4f} m'
        notes = (
            f'closure_error={closure_error:.4f}m, '
            f'final_yaw_error={final_yaw_error_deg:.2f}deg'
        )

        self.get_logger().info('===== Odom Out and Back Results =====')
        self.get_logger().info(f'Forward distance traveled: {self.forward_distance:.3f} m')
        self.get_logger().info(f'Rotation traveled: {self.rotation_deg:.2f} deg')
        self.get_logger().info(f'Return distance traveled: {self.return_distance:.3f} m')
        self.get_logger().info(f'Final closure error: {closure_error:.4f} m')
        self.get_logger().info(f'Final yaw error: {final_yaw_error_deg:.2f} deg')
        self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='odom_out_and_back',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting odom_out_and_back')
                rclpy.shutdown()
            return

        if time.time() - self.phase_start_time > MAX_TEST_TIME and self.phase != 'waiting_for_data':
            self.get_logger().error('Test timed out')
            self.publish()
            self.done = True
            self.finish_time = time.time()
            return

        if self.phase == 'waiting_for_data':
            if self.data_ready():
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_yaw = self.current_yaw

                self.phase_start_x = self.current_x
                self.phase_start_y = self.current_y
                self.phase_start_yaw = self.current_yaw

                self.phase = 'settling'
                self.phase_start_time = time.time()

                self.get_logger().info('Captured starting pose')
                self.get_logger().info(
                    f'Start -> x: {self.start_x:.3f}, y: {self.start_y:.3f}, '
                    f'yaw: {math.degrees(self.start_yaw):.2f} deg'
                )
                self.get_logger().info(f'Settling for {SETTLE_TIME:.1f} second(s)...')
            else:
                now = time.time()
                if now - self.wait_log_time > 1.0:
                    self.get_logger().info('Waiting for odom data...')
                    self.wait_log_time = now
            return

        if self.phase == 'settling':
            self.publish()
            if time.time() - self.phase_start_time >= SETTLE_TIME:
                self.set_new_phase('forward_1')
            return

        if self.phase == 'forward_1':
            dist = math.sqrt(
                (self.current_x - self.phase_start_x) ** 2 +
                (self.current_y - self.phase_start_y) ** 2
            )

            self.maybe_log_progress(
                f'Forward leg distance: {dist:.3f} m / {TARGET_DISTANCE:.3f} m'
            )

            if dist < TARGET_DISTANCE:
                self.publish(x=LINEAR_SPEED)
            else:
                self.publish()
                self.forward_distance = dist
                self.get_logger().info(
                    f'Forward leg complete. Distance traveled: {dist:.3f} m'
                )
                self.set_new_phase('rotate_180')
            return

        if self.phase == 'rotate_180':
            odom_delta_deg = abs(math.degrees(
                normalize_angle(self.current_yaw - self.phase_start_yaw)
            ))

            self.maybe_log_progress(
                f'Rotation progress: {odom_delta_deg:.1f} deg / {TARGET_ROTATION_DEG:.1f} deg'
            )

            remaining_error = TARGET_ROTATION_DEG - odom_delta_deg

            if remaining_error > 20.0:
                self.publish(z=ANGULAR_SPEED_FAST)
            elif remaining_error > ROTATION_STOP_TOLERANCE_DEG:
                self.publish(z=ANGULAR_SPEED_SLOW)
            else:
                self.publish()
                self.rotation_deg = odom_delta_deg
                self.get_logger().info(
                    f'Rotation complete. Odom: {odom_delta_deg:.1f} deg'
                )
                self.set_new_phase('forward_2')
            return

        if self.phase == 'forward_2':
            dist = math.sqrt(
                (self.current_x - self.phase_start_x) ** 2 +
                (self.current_y - self.phase_start_y) ** 2
            )

            self.maybe_log_progress(
                f'Return leg distance: {dist:.3f} m / {TARGET_DISTANCE:.3f} m'
            )

            if dist < TARGET_DISTANCE:
                self.publish(x=LINEAR_SPEED)
            else:
                self.publish()
                self.return_distance = dist
                self.get_logger().info(
                    f'Return leg complete. Distance traveled: {dist:.3f} m'
                )
                self.finish_and_exit()
            return


def main(args=None):
    rclpy.init(args=args)
    node = OdomOutAndBack()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()