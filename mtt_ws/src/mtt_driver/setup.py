import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mtt_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex',
    maintainer_email='alex.chorel-campanozzi@ecn.forces.gc.ca',
    description='ROS 2 driver for MTT-154 vehicle',
    license='Apalache Liscense 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mtt_driver_py = mtt_driver.mtt_driver:main',
            'mtt_ros_wrapper = mtt_driver.mtt_ros_wrapper:main',
            'mtt_teleop_joy = mtt_driver.mtt_teleop_joy:main',
        ],
    },
)
