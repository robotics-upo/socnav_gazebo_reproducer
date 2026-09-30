import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration

def generate_launch_description():

    

    # Launch configuration variables
    nvidia_gpu_arg = LaunchConfiguration('nvidia_gpu')
    declare_nvidia_gpu_cmd = DeclareLaunchArgument(
        'nvidia_gpu',
        default_value='true',
        description='Set to "false" to not use nvidia gpu.'
    )
    # to activate the use of nvidia gpu
    use_nvidia_gpu = [
        '__NV_PRIME_RENDER_OFFLOAD=1 ',
        '__GLX_VENDOR_LIBRARY_NAME=nvidia ',
    ]
    nvidia_gpu_prefix = IfElseSubstitution(
        condition=nvidia_gpu_arg,
        if_value=use_nvidia_gpu,
    )

    world_arg = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('socnav_gazebo_reproducer'), 'worlds', 'src1_01.world'),
        description='Full path to the world model file to load'
    )

    package_prefix = get_package_prefix('socnav_gazebo_reproducer')
    plugin_path = os.path.join(package_prefix, 'lib')
    gui_plugin = os.path.join(plugin_path, 'libAutoRecordGuiPlugin.so')
    furniture_model_path = os.path.join(
        get_package_share_directory('hunav_gazebo_wrapper'), 'models')
    gazebo_model_path = os.pathsep.join(
        path for path in (furniture_model_path, os.environ.get('GAZEBO_MODEL_PATH', ''))
        if path
    )
    gazebo_plugin_path = os.pathsep.join(
        path for path in (plugin_path, os.environ.get('GAZEBO_PLUGIN_PATH', ''))
        if path
    )

    # gazebo_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world_arg}.items(),
    #     condition=IfCondition(LaunchConfiguration('server'))
    # )

    # gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
    #     ),
    #     launch_arguments={
    #         'extra_gazebo_args': '--gui-client-plugin=' + gui_plugin,
    #     }.items(),
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )


    gzserver_cmd = [
        nvidia_gpu_prefix,
        'gzserver ',
        #'--pause ',
        # Pass through arguments to gzserver
         world_arg, 
        #_boolean_command('verbose'), '',
        '-s ', 'libgazebo_ros_init.so',
        '-s ', 'libgazebo_ros_factory.so',
        #'-s ', #'libgazebo_ros_state.so',
        #'--ros-args',
        #'--params-file', config_file,
    ]

    gzclient_cmd = [
        nvidia_gpu_prefix,
        'gzclient',
        #_boolean_command('verbose'), ' ',
        '--gui-client-plugin=' + gui_plugin,
    ]

    gzserver_process = ExecuteProcess(
        cmd=gzserver_cmd,
        output='screen',
        #additional_env=env,
        shell=True,
        on_exit=Shutdown(),
        condition=IfCondition(LaunchConfiguration('server')),
    )

    gzclient_process = ExecuteProcess(
        cmd=gzclient_cmd,
        output='screen',
        #additional_env=env,
        shell=True,
        on_exit=Shutdown(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )


    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_nvidia_gpu_cmd)
    ld.add_action(DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run headless.'))
    ld.add_action(DeclareLaunchArgument(
        'server', default_value='true', description='Set to "false" not to run gzserver.'))
    ld.add_action(SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', value=gazebo_plugin_path))
    ld.add_action(SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=gazebo_model_path))
    #ld.add_action(gazebo_server)
    #ld.add_action(gazebo_client)
    ld.add_action(gzserver_process)
    ld.add_action(gzclient_process)

    return ld
