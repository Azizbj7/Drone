import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('drone')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Drone_assembly.urdf')  
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'my_config.rviz')
    map_file = os.path.join(pkg_share, 'map', 'my_map.yaml')
    robot_description = None
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_file,
            description='urdf/Drone_assembly.urdf'
        ),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
         launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        
        launch_ros.actions.Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'image_size': [640,480],
                'time_per_frame': [1, 6],
                'camera_frame_id': 'camera_link_optical'
                }]
                ),
        #launch_ros.actions.Node(
         #   package='nav2_map_server',
          #  executable='map_server',
          #  name='map_server',
           # output='screen',
          #  parameters=[{'yaml_filename': map_file}]
           # ),
        launch_ros.actions.Node(
        package='drone', # Le package de votre noeud keyboard_teleop
        executable='keyboard_teleop', # Le nom de l'exécutable
        name='keyboard_teleop_node',
        output='screen',
        prefix='xterm -e',
    ),
    ])



 
