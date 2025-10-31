# src/sensor_bringup/setup.py
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='j1113019',
    maintainer_email='j1113019@naver.com',
    description='Sensor TF and topic management layer for LiDAR, IMU, and camera devices.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_normalizer = sensor_bringup.scan_normalizer:main',
        ],
    },
)
