from setuptools import find_packages, setup

package_name = 'suctionnet_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinkai',
    maintainer_email='jinkaiq@andrew.cmu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'suctionnet = suctionnet_ros2.suctionnet:main',
            'suctionnet_client = suctionnet_ros2.suctionnet_client:main',
            'dummy_segment = suctionnet_ros2.dummy_seg_mask:main'
        ],
    },
)
