import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch configuration variables
    world_arg = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('socnav_gazebo_reproducer'), 'worlds', '001.world'),
        description='Full path to the world model file to load'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_arg}.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(gazebo)

    return ld
