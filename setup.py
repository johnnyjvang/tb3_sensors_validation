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
            # Added to print and reset json output
            'reset_results = tb3_sensors_validation.reset_results:main',
            'summary_report = tb3_sensors_validation.summary_report:main',
        ],
    },
)
