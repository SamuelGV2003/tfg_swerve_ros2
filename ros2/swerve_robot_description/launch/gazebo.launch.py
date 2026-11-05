"""Simulación del robot en Gazebo Sim con ros2_control.

Abre Gazebo con un mundo vacío, agrega el robot y arranca los tres
controladores de controllers.yaml (joint_state_broadcaster,
steer_position_controller y wheel_velocity_controller). Este archivo no
mueve el robot; para eso se usan los launch de swerve_control.

Uso:
    ros2 launch swerve_robot_description gazebo.launch.py

Prueba manual de un controlador (gira ambos módulos 0.3 rad):
    ros2 topic pub /steer_position_controller/commands \
        std_msgs/msg/Float64MultiArray "data: [0.3, 0.3]"

Argumentos (también funcionan desde los launch de swerve_control, que
incluyen este archivo):
    usar_mallas:=true|false          mallas visuales o formas simples (true)
    balines_delanteros:=true|false   con o sin apoyos delanteros (true)

Ejemplo sin apoyos delanteros:
    ros2 launch swerve_robot_description gazebo.launch.py balines_delanteros:=false
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("swerve_robot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    xacro_path = os.path.join(pkg_share, "urdf", "swerve_robot.urdf.xacro")

    # gz_ros2_control no lee bien el archivo de controladores cuando su ruta
    # es larga, como la que queda dentro de install/ del workspace (falla
    # con "Couldn't parse params file"). Es un error conocido del paquete:
    # https://github.com/ros-controls/gz_ros2_control/issues/512
    # Para evitarlo, el archivo se copia a una ruta corta en /tmp.
    controllers_yaml_original = os.path.join(pkg_share, "config", "controllers.yaml")
    controllers_yaml_path = "/tmp/swerve_controllers.yaml"
    shutil.copy(controllers_yaml_original, controllers_yaml_path)

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", xacro_path,
                " controllers_yaml:=", controllers_yaml_path,
                " usar_mallas:=", LaunchConfiguration("usar_mallas"),
                " balines_delanteros:=", LaunchConfiguration("balines_delanteros"),
            ]),
            value_type=str,
        )
    }

    # Argumentos que se pasan al xacro. Los valores por defecto son los
    # mismos del xacro.
    usar_mallas_arg = DeclareLaunchArgument(
        "usar_mallas", default_value="true",
        description="Usar las mallas visuales (false: formas simples)",
    )
    balines_delanteros_arg = DeclareLaunchArgument(
        "balines_delanteros", default_value="true",
        description="Incluir los apoyos delanteros del prototipo",
    )

    # Mundo vacío. -r inicia la simulación sin pausa.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # Publica el modelo en /robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Crea el robot en Gazebo a partir de /robot_description. Aparece 5 cm
    # por encima del piso y cae hasta apoyarse.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "swerve_robot", "-z", "0.05"],
        output="screen",
    )

    # Pasa el reloj de Gazebo a ROS 2 (/clock)
    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Controladores definidos en config/controllers.yaml
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    steer_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steer_position_controller"],
        output="screen",
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller"],
        output="screen",
    )

    return LaunchDescription([
        usar_mallas_arg,
        balines_delanteros_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge_clock,
        joint_state_broadcaster_spawner,
        steer_controller_spawner,
        wheel_controller_spawner,
    ])
