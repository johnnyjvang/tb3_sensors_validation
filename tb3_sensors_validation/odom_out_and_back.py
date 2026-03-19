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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tb3_sensors_validation.result_utils import append_result


# ===== Test Settings =====
ODOM_TOPIC = '/odom'
CMD_VEL_TOPIC = '/cmd_vel'

TARGET_DISTANCE = 1.0          # meters forward, then 1.0 meter back
LINEAR_SPEED = 0.10            # m/s
ANGULAR_SPEED = 0.40           # rad/s

CONTROL_PERIOD = 0.05          # seconds
SETTLE_TIME = 1.0              # seconds after each phase
MAX_TEST_TIME = 40.0           # safety timeout

# Thresholds
PASS_CLOSURE_ERROR = 0.05      # meters
WARN_CLOSURE_ERROR = 0.10      # meters

PASS_FINAL_YAW_ERROR_DEG = 5.0
WARN_FINAL_YAW_ERROR_DEG = 10.0


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

        # Publisher / subscriber
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        # Timing / state
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        # Odom tracking
        self.msg_count = 0
        self.start_pose = None
        self.current_pose = None

        # Phase reference poses
        self.forward_start_pose = None
        self.rotation_start_yaw = None
        self.return_start_pose = None
        self.final_pose = None

        # Motion state
        self.phase = 'wait_for_odom'
        self.stop_time = None

        # Timers
        self.timer = self.create_timer(CONTROL_PERIOD, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting odom out-and-back test on topic: {ODOM_TOPIC}')
        self.get_logger().info(f'Publishing velocity commands on: {CMD_VEL_TOPIC}')
        self.get_logger().info(f'Target forward distance: {TARGET_DISTANCE:.2f} m')
        self.get_logger().info('Sequence: forward -> rotate 180 deg -> return')

    def odom_cb(self, msg):
        """
        Track current odom pose.
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

        if self.start_pose is None:
            self.start_pose = pose
            self.get_logger().info('First odom message received')

        self.current_pose = pose
        self.msg_count += 1

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
        Print live progress for the current test phase.
        """
        if self.done:
            return

        elapsed = time.time() - self.start_time

        progress_text = ''
        if self.phase == 'forward' and self.forward_start_pose is not None and self.current_pose is not None:
            dist = planar_distance(
                self.forward_start_pose['x'], self.forward_start_pose['y'],
                self.current_pose['x'], self.current_pose['y']
            )
            progress_text = f'forward distance: {dist:.3f} m'

        elif self.phase == 'rotating' and self.rotation_start_yaw is not None and self.current_pose is not None:
            turned_deg = abs(math.degrees(angle_diff_rad(self.current_pose['yaw'], self.rotation_start_yaw)))
            progress_text = f'rotation: {turned_deg:.2f} deg'

        elif self.phase == 'return' and self.return_start_pose is not None and self.current_pose is not None:
            dist = planar_distance(
                self.return_start_pose['x'], self.return_start_pose['y'],
                self.current_pose['x'], self.current_pose['y']
            )
            progress_text = f'return distance: {dist:.3f} m'

        else:
            progress_text = 'waiting'

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {MAX_TEST_TIME:.1f}s | '
            f'phase: {self.phase} | '
            f'messages: {self.msg_count} | '
            f'{progress_text}'
        )

    def finish_and_exit(self, status_override=None, notes_override=None):
        """
        Compute final closure error, save result, and prepare shutdown.
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
            closure_error = planar_distance(
                self.start_pose['x'], self.start_pose['y'],
                self.final_pose['x'], self.final_pose['y']
            )
            final_yaw_error_deg = abs(
                math.degrees(angle_diff_rad(self.final_pose['yaw'], self.start_pose['yaw']))
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

            self.get_logger().info('=== Odom Out-and-Back Results ===')
            self.get_logger().info(f'Topic: {ODOM_TOPIC}')
            self.get_logger().info(f'Total messages: {self.msg_count}')
            self.get_logger().info(f'Closure error: {closure_error:.4f} m')
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
        """
        Main state machine for the out-and-back test.
        """
        now = time.time()
        elapsed = now - self.start_time

        if self.done:
            if now - self.finish_time > 0.5:
                self.get_logger().info('Exiting odom_out_and_back')
                rclpy.shutdown()
            return

        if elapsed > MAX_TEST_TIME:
            self.finish_and_exit(
                status_override='FAIL',
                notes_override='out-and-back test timed out'
            )
            return

        if self.phase == 'wait_for_odom':
            if self.start_pose is not None and self.current_pose is not None:
                self.forward_start_pose = dict(self.current_pose)
                self.phase = 'forward'
                self.get_logger().info('Starting forward leg...')
            return

        if self.phase == 'forward':
            dist = planar_distance(
                self.forward_start_pose['x'], self.forward_start_pose['y'],
                self.current_pose['x'], self.current_pose['y']
            )

            if dist < TARGET_DISTANCE:
                self.publish_cmd(LINEAR_SPEED, 0.0)
            else:
                self.stop_robot()
                self.stop_time = now
                self.phase = 'forward_settle'
                self.get_logger().info('Forward leg complete, settling...')
            return

        if self.phase == 'forward_settle':
            self.stop_robot()
            if now - self.stop_time >= SETTLE_TIME:
                self.rotation_start_yaw = self.current_pose['yaw']
                self.phase = 'rotating'
                self.get_logger().info('Starting 180-degree rotation...')
            return

        if self.phase == 'rotating':
            turned_deg = abs(math.degrees(angle_diff_rad(self.current_pose['yaw'], self.rotation_start_yaw)))

            if turned_deg < 180.0:
                self.publish_cmd(0.0, ANGULAR_SPEED)
            else:
                self.stop_robot()
                self.stop_time = now
                self.phase = 'rotation_settle'
                self.get_logger().info('Rotation complete, settling...')
            return

        if self.phase == 'rotation_settle':
            self.stop_robot()
            if now - self.stop_time >= SETTLE_TIME:
                self.return_start_pose = dict(self.current_pose)
                self.phase = 'return'
                self.get_logger().info('Starting return leg...')
            return

        if self.phase == 'return':
            dist = planar_distance(
                self.return_start_pose['x'], self.return_start_pose['y'],
                self.current_pose['x'], self.current_pose['y']
            )

            if dist < TARGET_DISTANCE:
                self.publish_cmd(LINEAR_SPEED, 0.0)
            else:
                self.stop_robot()
                self.stop_time = now
                self.phase = 'final_settle'
                self.get_logger().info('Return leg complete, settling...')
            return

        if self.phase == 'final_settle':
            self.stop_robot()
            if now - self.stop_time >= SETTLE_TIME:
                self.final_pose = dict(self.current_pose)
                self.finish_and_exit()
            return


def main(args=None):
    rclpy.init(args=args)
    node = OdomOutAndBack()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()