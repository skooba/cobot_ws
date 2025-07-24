"""Setup configuration for cobot_control_pkg."""
from setuptools import find_packages, setup

package_name = 'cobot_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'smach', 'smach_ros'],
    zip_safe=True,
    maintainer='jacob',
    maintainer_email='jarlaufer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proximity_sensor = cobot_control_pkg.proximity_sensor:main',
            'emergency_stop = cobot_control_pkg.emergency_stop:main',
            'speed_control = cobot_control_pkg.speed_control:main',
            'ur_robot_controller = cobot_control_pkg.ur_robot_controller:main',
        ],
    },
)
