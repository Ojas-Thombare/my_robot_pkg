import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch argument
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gz sim World'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')

    pkg_share = get_package_share_directory('my_robot_pkg')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    # Set gazebo sim resource path
    gazebo_resource_path = ExecuteProcess(
        cmd=['export', 'GZ_SIM_RESOURCE_PATH=' + os.path.join(pkg_share, 'worlds')],
        output='screen'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', [world, '.sdf', ' -v 4', ' -r'])],
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read(), 'use_sim_time': use_sim_time}]
    )

    # Spawn entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', urdf_file,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '3.1415',
                   '-name', 'my_robot',
                   '-allow_renaming', 'false'],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/my_robot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        gazebo_resource_path,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
    ])
