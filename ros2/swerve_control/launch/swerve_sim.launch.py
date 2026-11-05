"""Simulación del robot controlado por velocidad (/cmd_vel).

Incluye gazebo.launch.py de swerve_robot_description y arranca el nodo de
cinemática inversa, que convierte /cmd_vel en comandos para los módulos.

Uso:
    ros2 launch swerve_control swerve_sim.launch.py

Acepta los argumentos de gazebo.launch.py (usar_mallas, balines_delanteros).

Ejemplos desde otra terminal. Se publica a 10 Hz (-r 10) porque el nodo
detiene el robot si pasa más de 0.5 s sin recibir /cmd_vel:
    Avance recto durante 5 s
        timeout 5 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.3}"
    Rotación en el lugar
        timeout 5 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "angular: {z: 1.0}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_swerve_description = get_package_share_directory('swerve_robot_description')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_swerve_description, 'launch', 'gazebo.launch.py')
        )
    )

    kinematics_node = Node(
        package='swerve_control',
        executable='swerve_kinematics_node',
        name='swerve_kinematics_node',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        kinematics_node,
    ])