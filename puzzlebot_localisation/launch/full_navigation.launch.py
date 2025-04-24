#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al directorio share de tu paquete
    pkg_share = get_package_share_directory('puzzlebot_localisation')

    # Ruta al archivo RViz
    rviz_config = os.path.join(pkg_share, 'config', 'puzzlebot_visualization.rviz')

    return LaunchDescription([

        # Nodo de Odometría
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_odometry',
            name='puzzlebot_odometry',
            output='screen'
        ),

        # Nodo de Control (PI + feed-forward)
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_controller',
            name='puzzlebot_controller',
            output='screen'
        ),

        # Nodo Generador de Rutas (lee waypoints.yaml con tiempos)
        Node(
            package='puzzlebot_localisation',
            executable='puzzlebot_path_generator',
            name='puzzlebot_path_generator',
            output='screen'
        ),

    ])
