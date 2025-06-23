from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_pkg')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    print(f"Loading URDF from: {urdf_file}")  # Debug print
    if not os.path.exists(urdf_file):
        print(f"Error: URDF file not found at {urdf_file}")
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
