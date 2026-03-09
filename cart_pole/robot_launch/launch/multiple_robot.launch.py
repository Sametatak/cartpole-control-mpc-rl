import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_path = "/home/atak/itu_ws/src/cart_pole" 
    urdf_file = os.path.join(pkg_path, "robot_description/urdf/cart_pole.urdf")
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Gazebo Classic Başlat ve STATE PLUGIN'i Yükle (Sihirli satır eklendi)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '-s libgazebo_ros_state.so'}.items()
    )

    spawn_robots_cmds = []

    # 10 Tane Robot Oluştur
    for i in range(10):
        robot_name = f"cart_pole_{i}"
        namespace = f"robot_{i}"
        y_pos = float(i * 2.0) # Robotlar birbirine çarpmasın diye 2m arayla diz

        # Her robot için Namespace Grubu
        robot_group = GroupAction([
            PushRosNamespace(namespace),
            
            # Robot State Publisher (Her robotun kendi TF ağacı için)
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': True,
                    'frame_prefix': f"{namespace}/"
                }]
            ),

            # Robotu Spawn Et
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot_name, 
                    '-file', urdf_file, 
                    '-x', '0.0', 
                    '-y', str(y_pos), 
                    '-z', '0.1',
                    '-robot_namespace', namespace # Gazebo pluginleri bu NS'yi kullanır
                ],
                output='screen'
            ),
        ])
        
        spawn_robots_cmds.append(robot_group)

    return LaunchDescription([
        gazebo,
        *spawn_robots_cmds
    ])
