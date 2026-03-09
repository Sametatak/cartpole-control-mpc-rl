import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = "/home/atak/itu_ws/src/cart_pole" # Yolunu kontrol et
    urdf_file = os.path.join(pkg_path, "robot_description/urdf/cart_pole.urdf")
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Gazebo Classic Başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 2. Robotu Spawn Et
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'cart_pole', '-file', urdf_file, '-x', '0', '-z', '0.1'],
        output='screen'
    )

    # 3. State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher
    ])
