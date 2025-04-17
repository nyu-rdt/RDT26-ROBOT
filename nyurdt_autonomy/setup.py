from setuptools import setup

package_name = 'nyurdt_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'sensor_service=nyurdt_autonomy.sensor_service:main',
            'locomotion=nyurdt_autonomy.locomotion:main',
            'excavation=nyurdt_autonomy.excavation:main',
            'autonomy_destroyer=nyurdt_autonomy.autonomy_destroyer:main',
	    'camera_subscriber=nyurdt_autonomy.webcam_subscriber:main'
        ],
    },
)
