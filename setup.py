from setuptools import setup
import os

package_name = 'ros_openimu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='mantacast4154@gmail.com',
    description='Aceinna IMU to ROS2 Driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sr_imu = ros_openimu.openimu_driver:main"
        ],
    },
)