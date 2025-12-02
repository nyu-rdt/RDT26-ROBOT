from setuptools import find_packages, setup

package_name = 'april'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/camera_apriltag.launch.yml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mandy',
    maintainer_email='mandy@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "april_tf=april.apriltag_tf:main",
        ],
    },
)
