"""Simulación del robot con el controlador de punto objetivo.

Incluye gazebo.launch.py de swerve_robot_description y arranca
swerve_punto_objetivo_node, que recorre una trayectoria de puntos.

Este launch no arranca swerve_kinematics_node, porque el controlador de
punto objetivo hace su propia cinemática inversa y le envía los comandos
directamente a los controladores de ros2_control. No se debe correr al
mismo tiempo que swerve_sim.launch.py, porque los dos nodos publicarían en
los mismos tópicos.

Uso:
    ros2 launch swerve_control swerve_punto_objetivo.launch.py
    ros2 launch swerve_control swerve_punto_objetivo.launch.py trayectoria:=triangulo
    ros2 launch swerve_control swerve_punto_objetivo.launch.py trayectoria:="0.3 0.2 90"
    ros2 launch swerve_control swerve_punto_objetivo.launch.py usar_dither:=true

Acepta también los argumentos de gazebo.launch.py (usar_mallas,
balines_delanteros). Los demás parámetros del nodo se cambian lanzándolo
con ros2 run (ver la documentación del nodo).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_swerve_description = get_package_share_directory('swerve_robot_description')

    trayectoria_arg = DeclareLaunchArgument(
        'trayectoria', default_value='cuadrado',
        description='cuadrado, triángulo, o puntos propios "x y theta_grados ..."')
    dither_arg = DeclareLaunchArgument(
        'usar_dither', default_value='false',
        description='Dither de dirección del prototipo (apagado por defecto)')
    espera_arg = DeclareLaunchArgument(
        'espera_inicial', default_value='8.0',
        description='Segundos de simulación antes de arrancar (margen para grabar en Gazebo)')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_swerve_description, 'launch', 'gazebo.launch.py')
        )
    )

    punto_objetivo_node = Node(
        package='swerve_control',
        executable='swerve_punto_objetivo_node',
        name='swerve_punto_objetivo_node',
        output='screen',
        parameters=[{
            'trayectoria': ParameterValue(LaunchConfiguration('trayectoria'), value_type=str),
            'usar_dither': ParameterValue(LaunchConfiguration('usar_dither'), value_type=bool),
            'espera_inicial': ParameterValue(LaunchConfiguration('espera_inicial'), value_type=float),
        }],
    )

    return LaunchDescription([
        trayectoria_arg,
        dither_arg,
        espera_arg,
        gazebo,
        punto_objetivo_node,
    ])
