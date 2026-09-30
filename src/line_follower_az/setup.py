import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'line_follower_az'


def data_files_for(folder, pattern='*'):
    files = glob(os.path.join(folder, pattern))
    return (os.path.join('share', package_name, folder), files)


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        data_files_for('launch', '*.py'),
        data_files_for('config', '*.yaml'),
        data_files_for('description', '*.urdf'),
        data_files_for('worlds', '*.world'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Priyanshu Rout',
    maintainer_email='priyanshurout8969@gmail.com',
    description='ROS2-Based Line Follower 2.0 (simulation-first, with headless 2D simulator)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = line_follower_az.sensor_node:main',
            'pid_controller_node = line_follower_az.pid_controller_node:main',
            'motor_driver_node = line_follower_az.motor_driver_node:main',
            'logger_node = line_follower_az.logger_node:main',
            'monitor_node = line_follower_az.monitor_node:main',
            'line_follower_sim = line_follower_az.simulator:main',
        ],
    },
)
