from setuptools import find_packages, setup

package_name = 'seg_mask'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    py_modules=[
        'seg_mask.yolo_sam2'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gaurav',
    maintainer_email='gaurav.sethia08@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'yolo_sam2 = seg_mask.yolo_sam2:main',
        'yolo_sam2_test = seg_mask.test_service:main'
        ],
    },
)
