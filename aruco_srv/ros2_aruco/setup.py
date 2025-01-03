from setuptools import setup
import os
from glob import glob

package_name = 'ros2_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Sprague',
    maintainer_email='nathan.r.sprague@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = ros2_aruco.aruco_node:main',
            'aruco_generate_marker = ros2_aruco.aruco_generate_marker:main',
            'aruco_pose_server = ros2_aruco.aruco_pose_server:main',
            'wait_for_msg = ros2_aruco.wait_for_msg:main',
            'get_drop_pose = ros2_aruco.get_drop_pose:main',
            'test_drop_pose = ros2_aruco.test_drop_pose:main',
            'find_aruco = ros2_aruco.find_aruco:main'
        ],
    },
)
