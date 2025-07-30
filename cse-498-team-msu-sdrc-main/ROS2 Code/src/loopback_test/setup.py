from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'loopback_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'vesc_msgs'],
    zip_safe=True,
    maintainer='Toby Wright',
    maintainer_email='wrigh851@msu.edu',
    description='ROS 2 Controller Receiver Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loopback_test_laptop = loopback_test.loopback_test_laptop:main',
            'loopback_test_car = loopback_test.loopback_test_car:main',
        ],
    },
)
