from setuptools import setup

import os
from glob import glob

package_name = 'nyurdt_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nyu-rdt',
    maintainer_email='eng-rdt@nyu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_subscriber=nyurdt_control.command_subscriber:main',
            'i2c_reader=nyurdt_control.i2c_reader:main',
            'cmd_vel_bridge=nyurdt_control.cmd_vel_bridge:main'
        ],
    },
)