"""
sensors_validation_all.launch.py

Run all TurtleBot3 sensor validation tests sequentially
and print a final summary report at the end.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # ===== Reset results =====
    reset_results = Node(
        package='tb3_sensors_validation',
        executable='reset_results',
        name='reset_results',
        output='screen'
    )

    # ===== Message rate =====
    message_rate_check = Node(
        package='tb3_sensors_validation',
        executable='message_rate_check',
        name='message_rate_check',
        output='screen'
    )

    # ===== Odom tests =====
    odom_stationary_drift = Node(
        package='tb3_sensors_validation',
        executable='odom_stationary_drift',
        name='odom_stationary_drift',
        output='screen'
    )

    odom_forward_accuracy = Node(
        package='tb3_sensors_validation',
        executable='odom_forward_accuracy',
        name='odom_forward_accuracy',
        output='screen'
    )

    odom_backward_accuracy = Node(
        package='tb3_sensors_validation',
        executable='odom_backward_accuracy',
        name='odom_backward_accuracy',
        output='screen'
    )

    odom_rotation_accuracy = Node(
        package='tb3_sensors_validation',
        executable='odom_rotation_accuracy',
        name='odom_rotation_accuracy',
        output='screen'
    )

    odom_out_and_back = Node(
        package='tb3_sensors_validation',
        executable='odom_out_and_back',
        name='odom_out_and_back',
        output='screen'
    )

    # ===== IMU tests =====
    imu_gyro_bias = Node(
        package='tb3_sensors_validation',
        executable='imu_gyro_bias',
        name='imu_gyro_bias',
        output='screen'
    )

    imu_accel_noise = Node(
        package='tb3_sensors_validation',
        executable='imu_accel_noise',
        name='imu_accel_noise',
        output='screen'
    )

    imu_yaw_drift = Node(
        package='tb3_sensors_validation',
        executable='imu_yaw_drift',
        name='imu_yaw_drift',
        output='screen'
    )

    imu_rotation_response = Node(
        package='tb3_sensors_validation',
        executable='imu_rotation_response',
        name='imu_rotation_response',
        output='screen'
    )

    imu_odom_agreement = Node(
        package='tb3_sensors_validation',
        executable='imu_odom_agreement',
        name='imu_odom_agreement',
        output='screen'
    )

    # ===== LiDAR tests =====
    lidar_valid_ranges = Node(
        package='tb3_sensors_validation',
        executable='lidar_valid_ranges',
        name='lidar_valid_ranges',
        output='screen'
    )

    lidar_stationary_noise = Node(
        package='tb3_sensors_validation',
        executable='lidar_stationary_noise',
        name='lidar_stationary_noise',
        output='screen'
    )

    lidar_front_obstacle_accuracy = Node(
        package='tb3_sensors_validation',
        executable='lidar_front_obstacle_accuracy',
        name='lidar_front_obstacle_accuracy',
        output='screen'
    )

    lidar_nearby_object_stability = Node(
        package='tb3_sensors_validation',
        executable='lidar_nearby_object_stability',
        name='lidar_nearby_object_stability',
        output='screen'
    )

    # ===== Summary =====
    summary_report = Node(
        package='tb3_sensors_validation',
        executable='summary_report',
        name='summary_report',
        output='screen'
    )

    return LaunchDescription([
        reset_results,

        RegisterEventHandler(
            OnProcessExit(
                target_action=reset_results,
                on_exit=[TimerAction(period=1.0, actions=[message_rate_check])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=message_rate_check,
                on_exit=[TimerAction(period=1.0, actions=[odom_stationary_drift])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=odom_stationary_drift,
                on_exit=[TimerAction(period=1.0, actions=[odom_forward_accuracy])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=odom_forward_accuracy,
                on_exit=[TimerAction(period=1.0, actions=[odom_backward_accuracy])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=odom_backward_accuracy,
                on_exit=[TimerAction(period=1.0, actions=[odom_rotation_accuracy])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=odom_rotation_accuracy,
                on_exit=[TimerAction(period=1.0, actions=[odom_out_and_back])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=odom_out_and_back,
                on_exit=[TimerAction(period=1.0, actions=[imu_gyro_bias])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_gyro_bias,
                on_exit=[TimerAction(period=1.0, actions=[imu_accel_noise])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_accel_noise,
                on_exit=[TimerAction(period=1.0, actions=[imu_yaw_drift])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_yaw_drift,
                on_exit=[TimerAction(period=1.0, actions=[imu_rotation_response])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_rotation_response,
                on_exit=[TimerAction(period=1.0, actions=[imu_odom_agreement])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_odom_agreement,
                on_exit=[TimerAction(period=1.0, actions=[lidar_valid_ranges])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=lidar_valid_ranges,
                on_exit=[TimerAction(period=1.0, actions=[lidar_stationary_noise])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=lidar_stationary_noise,
                on_exit=[TimerAction(period=1.0, actions=[lidar_front_obstacle_accuracy])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=lidar_front_obstacle_accuracy,
                on_exit=[TimerAction(period=1.0, actions=[lidar_nearby_object_stability])]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=lidar_nearby_object_stability,
                on_exit=[TimerAction(period=1.0, actions=[summary_report])]
            )
        ),
    ])