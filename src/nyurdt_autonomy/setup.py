from setuptools import find_packages, setup

package_name = 'nyurdt_autonomy'

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
    maintainer='nyu-rdt',
    maintainer_email='ag8754@nyu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_bridge = nyurdt_autonomy.serial_bridge:main',
            'LocomotionNode = nyurdt_autonomy.locomotion:main',
            'ExcavationNode = nyurdt_autonomy.excavation:main',
            'ConstructionNode = nyurdt_autonomy.construction:main',
            'autonomy_destroyer = nyurdt_autonomy.autonomy_destroyer:main',
            'autonomy_manager = nyurdt_autonomy.autonomy_manager:main'
        ],
    },
)
