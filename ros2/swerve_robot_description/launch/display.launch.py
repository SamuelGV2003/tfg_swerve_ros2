"""Visualización del modelo en RViz, sin simulación física.

Lanza robot_state_publisher, joint_state_publisher_gui y RViz2. Sirve para
revisar el modelo antes de pasarlo a Gazebo: con los deslizadores de
joint_state_publisher_gui se mueven las juntas de dirección y tracción de
cada módulo y se comprueba que giren sobre el eje correcto.

Uso:
    ros2 launch swerve_robot_description display.launch.py

Argumentos:
    usar_mallas:=true|false          mallas visuales o formas simples (true)
    balines_delanteros:=true|false   con o sin apoyos delanteros (true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("swerve_robot_description")
    xacro_path = os.path.join(pkg_share, "urdf", "swerve_robot.urdf.xacro")
    rviz_config_path = os.path.join(pkg_share, "config", "swerve_robot.rviz")

    # value_type=str evita que el launch intente leer el URDF como YAML.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", xacro_path,
                " usar_mallas:=", LaunchConfiguration("usar_mallas"),
                " balines_delanteros:=", LaunchConfiguration("balines_delanteros"),
            ]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "usar_mallas", default_value="true",
            description="Usar las mallas visuales (false: formas simples)",
        ),
        DeclareLaunchArgument(
            "balines_delanteros", default_value="true",
            description="Incluir los apoyos delanteros del prototipo",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
        ),
    ])
