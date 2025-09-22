from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xacro_path = '/workspace/src/multi_model_sensor_fusion/xacro/my_robot.xacro'

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            )
        ),

        # Convert Xacro to URDF and publish robot_state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'),
                    ' ',
                    xacro_path
                ])
            }]
        ),

        # Spawn robot in Gazebo using robot_description
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-topic', 'robot_description',
                '-entity', 'my_robot'
            ],
            output='screen'
        )
    ])
