import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch configuration variables
    world_arg = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('socnav_gazebo_reproducer'), 'worlds', 'src1_01.world'),
        description='Full path to the world model file to load'
    )

    package_prefix = get_package_prefix('socnav_gazebo_reproducer')
    plugin_path = os.path.join(package_prefix, 'lib')
    gui_plugin = os.path.join(plugin_path, 'libAutoRecordGuiPlugin.so')
    gazebo_plugin_path = os.pathsep.join(
        path for path in (plugin_path, os.environ.get('GAZEBO_PLUGIN_PATH', ''))
        if path
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_arg}.items(),
        condition=IfCondition(LaunchConfiguration('server'))
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={
            'extra_gazebo_args': '--gui-client-plugin=' + gui_plugin,
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run headless.'))
    ld.add_action(DeclareLaunchArgument(
        'server', default_value='true', description='Set to "false" not to run gzserver.'))
    ld.add_action(SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', value=gazebo_plugin_path))
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    return ld
