from setuptools import setup
from glob import glob
import os

package_name = 'nyurdt_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
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
            "apriltag_detector = nyurdt_localization.apriltag_detector:main",
            "calibrate_camera=nyurdt_localization.calibrate_camera:main",
            'rplidar = nyurdt_localization.lidar_publisher_node:main',
        ],
    },
)
