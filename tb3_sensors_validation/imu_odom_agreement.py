"""
imu_odom_agreement.py

Command the robot to rotate in place and compare the yaw change measured
by the IMU and odometry.

Goal:
- Command a fixed in-place rotation
- Measure IMU-reported yaw change
- Measure odom-reported yaw change
- Compare the two measurements
- Save the result to the CSV results file
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
IMU_TOPIC = '/imu'
ODOM_TOPIC = '/odom'
CMD_VEL_TOPIC = '/cmd_vel'

TARGET_ROTATION_DEG = 90.0
ANGULAR_SPEED = 0.4
CONTROL_PERIOD = 0.05
SETTLE_TIME = 1.0
MAX_TEST_TIME = 10.0

# Thresholds
PASS_AGREEMENT_ERROR_DEG = 5.0
WARN_AGREEMENT_ERROR_DEG = 10.0


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


class ImuOdomAgreement(Node):
    def __init__(self):
        super().__init__('imu_odom_agreement')

        # Publisher / subscribers
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.imu_sub = self.create_subscription(Imu, IMU_TOPIC, self.imu_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        # Timing / state
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        # IMU tracking
        self.imu_msg_count = 0
        self.imu_start_yaw = None
        self.imu_current_yaw = None
        self.imu_final_yaw = None

        # Odom tracking
        self.odom_msg_count = 0
        self.odom_start_yaw = None
        self.odom_current_yaw = None
        self.odom_final_yaw = None

        # Motion state
        self.phase = 'wait_for_data'
        self.stop_time = None

        # Timers
        self.timer = self.create_timer(CONTROL_PERIOD, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info('Starting IMU/Odom agreement test')
        self.get_logger().info(f'IMU topic: {IMU_TOPIC}')
        self.get_logger().info(f'Odom topic: {ODOM_TOPIC}')
        self.get_logger().info(f'Cmd_vel topic: {CMD_VEL_TOPIC}')
        self.get_logger().info(f'Target rotation: {TARGET_ROTATION_DEG:.1f} deg')

    def imu_cb(self, msg):
        """
        Track current IMU yaw.
        """
        q = msg.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.imu_start_yaw is None:
            self.imu_start_yaw = yaw
            self.get_logger().info('First IMU message received')

        self.imu_current_yaw = yaw
        self.imu_msg_count += 1

    def odom_cb(self, msg):
        """
        Track current odom yaw.
        """
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.odom_start_yaw is None:
            self.odom_start_yaw = yaw
            self.get_logger().info('First odom message received')

        self.odom_current_yaw = yaw
        self.odom_msg_count += 1

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        """
        Publish a Twist command.
        """
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
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

        if self.imu_start_yaw is not None and self.imu_current_yaw is not None:
            imu_deg = math.degrees(angle_diff_rad(self.imu_current_yaw, self.imu_start_yaw))
        else:
            imu_deg = 0.0

        if self.odom_start_yaw is not None and self.odom_current_yaw is not None:
            odom_deg = math.degrees(angle_diff_rad(self.odom_current_yaw, self.odom_start_yaw))
        else:
            odom_deg = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {MAX_TEST_TIME:.1f}s | '
            f'phase: {self.phase} | '
            f'imu: {imu_deg:.2f} deg | '
            f'odom: {odom_deg:.2f} deg'
        )

    def finish_and_exit(self, status_override=None, notes_override=None):
        """
        Compute final agreement result, save CSV row, and prepare shutdown.
        """
        self.stop_robot()

        if status_override is not None:
            status = status_override
            measurement = '0.00 deg'
            notes = notes_override if notes_override else 'test aborted'
            self.get_logger().error(f'Test failed: {notes}')
        elif (
            self.imu_msg_count < 2 or
            self.odom_msg_count < 2 or
            self.imu_start_yaw is None or
            self.imu_final_yaw is None or
            self.odom_start_yaw is None or
            self.odom_final_yaw is None
        ):
            status = 'FAIL'
            measurement = '0.00 deg'
            notes = 'No sufficient IMU or odom messages received'
            self.get_logger().error('Test failed: insufficient IMU or odom messages received')
        else:
            imu_deg = math.degrees(angle_diff_rad(self.imu_final_yaw, self.imu_start_yaw))
            odom_deg = math.degrees(angle_diff_rad(self.odom_final_yaw, self.odom_start_yaw))
            agreement_error_deg = abs(abs(imu_deg) - abs(odom_deg))

            if agreement_error_deg <= PASS_AGREEMENT_ERROR_DEG:
                status = 'PASS'
            elif agreement_error_deg <= WARN_AGREEMENT_ERROR_DEG:
                status = 'WARN'
            else:
                status = 'FAIL'

            measurement = f'{agreement_error_deg:.2f} deg'
            notes = (
                f'imu_rotation={imu_deg:.2f}deg, '
                f'odom_rotation={odom_deg:.2f}deg, '
                f'agreement_error={agreement_error_deg:.2f}deg'
            )

            self.get_logger().info('=== IMU/Odom Agreement Results ===')
            self.get_logger().info(f'IMU measured rotation: {imu_deg:.2f} deg')
            self.get_logger().info(f'Odom measured rotation: {odom_deg:.2f} deg')
            self.get_logger().info(f'Agreement error: {agreement_error_deg:.2f} deg')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='imu_odom_agreement',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        """
        Main control loop for the agreement test.
        """
        now = time.time()
        elapsed = now - self.start_time

        if self.done:
            if now - self.finish_time > 0.5:
                self.get_logger().info('Exiting imu_odom_agreement')
                rclpy.shutdown()
            return

        if elapsed > MAX_TEST_TIME:
            self.finish_and_exit(
                status_override='FAIL',
                notes_override='IMU/odom agreement test timed out'
            )
            return

        if self.phase == 'wait_for_data':
            if (
                self.imu_start_yaw is not None and
                self.imu_current_yaw is not None and
                self.odom_start_yaw is not None and
                self.odom_current_yaw is not None
            ):
                self.phase = 'rotating'
                self.get_logger().info('Starting rotation...')
            return

        if self.phase == 'rotating':
            odom_deg = abs(math.degrees(angle_diff_rad(self.odom_current_yaw, self.odom_start_yaw)))

            if odom_deg < TARGET_ROTATION_DEG:
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
                self.imu_final_yaw = self.imu_current_yaw
                self.odom_final_yaw = self.odom_current_yaw
                self.finish_and_exit()
            return


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomAgreement()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()