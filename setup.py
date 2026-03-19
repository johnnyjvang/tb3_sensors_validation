from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tb3_sensors_validation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jvang',
    maintainer_email='johnnyjvang@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'message_rate_check = tb3_sensors_validation.message_rate_check:main',
            'odom_stationary_drift = tb3_sensors_validation.odom_stationary_drift:main',
            'odom_forward_accuracy = tb3_sensors_validation.odom_forward_accuracy:main',
            'odom_backward_accuracy = tb3_sensors_validation.odom_backward_accuracy:main',
            'odom_rotation_accuracy = tb3_sensors_validation.odom_rotation_accuracy:main',
            'odom_out_and_back = tb3_sensors_validation.odom_out_and_back:main',
            'imu_gyro_bias = tb3_sensors_validation.imu_gyro_bias:main',
            'imu_accel_noise = tb3_sensors_validation.imu_accel_noise:main',
            'imu_yaw_drift = tb3_sensors_validation.imu_yaw_drift:main',
            'imu_rotation_response = tb3_sensors_validation.imu_rotation_response:main',
            'imu_odom_agreement = tb3_sensors_validation.imu_odom_agreement:main',
            # Added to print and reset json output
            'reset_results = tb3_sensors_validation.reset_results:main',
            'summary_report = tb3_sensors_validation.summary_report:main',
        ],
    },
)
