from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Récupération du chemin du package 'drone'
    pkg_share = get_package_share_directory('drone')
    urdf_path = os.path.join(pkg_share, 'urdf', 'Drone_assembly.urdf')

    # Lecture du fichier URDF
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Lancer Gazebo avec le plugin ROS
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Publier la description du robot (TF, joints, etc.)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Spawner le robot dans Gazebo en utilisant le topic robot_description
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'drone',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        ),
    ])

