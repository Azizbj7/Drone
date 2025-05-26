from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('drone')
    urdf_path = os.path.join(pkg_share, 'urdf', 'Drone_assembly.urdf')
    
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # Chemin vers le lancement de Gazebo Fortress
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    return LaunchDescription([
        # Lancer Gazebo Fortress
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'verbose': 'true'}.items()
        ),

        # Publier la description du robot (TF, joints)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        
        # Spawn le robot dans Gazebo via robot_description
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
