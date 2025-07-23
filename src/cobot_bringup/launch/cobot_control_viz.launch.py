from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # Include UR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur5',
                'robot_ip': '192.168.56.101',
                'use_mock_hardware': 'true',
                'headless': 'true'
            }.items()
        ),
        
        # Delayed controller activation and cobot nodes
        # Wait for the UR controllers to initialize before activating the velocity controller
        # Then launch the cobot nodes after controller is ready
        TimerAction(
            period=8.0,  # 8 second delay
            actions=[
                # First activate the velocity controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['forward_velocity_controller', '--activate']
                ),
                # Then launch cobot nodes with additional delay to ensure controller is ready
                TimerAction(
                    period=2.0,  # Additional 2 second delay after controller activation
                    actions=[
                        Node(package='cobot_control_pkg', executable='proximity_sensor'),
                        Node(package='cobot_control_pkg', executable='emergency_stop'),
                        Node(package='cobot_control_pkg', executable='speed_control'),
                        Node(package='cobot_control_pkg', executable='ur_robot_controller'),
                    ]
                )
            ]
        ),
    ])