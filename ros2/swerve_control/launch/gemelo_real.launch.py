"""Sombra digital con el robot real en RViz.

Arranca robot_state_publisher, el puente al EV3 (gemelo_puente_node), RViz
y, si se pide, el nodo del joystick.

Uso:
  Con el robot real (antes hay que correr ./swerve_gemelo en el EV3):
    ros2 launch swerve_control gemelo_real.launch.py
  Sin joystick (se maneja con el teclado desde otra terminal):
    ros2 launch swerve_control gemelo_real.launch.py joystick:=false

Argumentos:
  ev3_ip              IP del EV3 (por defecto 192.168.1.103)
  joystick            true arranca joy_node
  usar_mallas         true usa las mallas visuales
  balines_delanteros  false quita la barra de apoyos delantera (por defecto
                      false, para que el modelo se vea más limpio)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RVIZ_CONFIG = """
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.1
      Plane Cell Count: 40
      Reference Frame: odom
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false
      Value: true
    - Class: rviz_default_plugins/Path
      Name: Recorrido
      Enabled: true
      Color: 255; 140; 0
      Line Style: Lines
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /gemelo/trayectoria
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
      Value: false
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.2
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.0
      Pitch: 0.6
      Yaw: 3.6
      Target Frame: base_footprint
      Value: Orbit (rviz)
    Saved: ~
"""


def es_true(texto):
    return str(texto).lower() in ('true', '1', 'si', 'yes')


def configurar(context):
    joystick = es_true(LaunchConfiguration('joystick').perform(context))
    ev3_ip = LaunchConfiguration('ev3_ip').perform(context)
    usar_mallas = LaunchConfiguration('usar_mallas').perform(context)
    balines = LaunchConfiguration('balines_delanteros').perform(context)

    # Modelo del robot (el mismo xacro de swerve_robot_description)
    descripcion = get_package_share_directory('swerve_robot_description')
    xacro_path = os.path.join(descripcion, 'urdf', 'swerve_robot.urdf.xacro')
    import xacro
    urdf = xacro.process_file(xacro_path, mappings={
        'usar_mallas': usar_mallas,
        'balines_delanteros': balines,
        'controllers_yaml': '/tmp/swerve_controllers.yaml',
    }).toxml()

    # Configuración de RViz en una ruta corta fija (igual que controllers.yaml)
    rviz_path = '/tmp/swerve_gemelo.rviz'
    with open(rviz_path, 'w') as f:
        f.write(RVIZ_CONFIG)

    acciones = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}], output='screen'),
        Node(package='swerve_control', executable='gemelo_puente_node',
             name='gemelo_puente', output='screen',
             parameters=[{'ev3_ip': ev3_ip}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_path]),
    ]
    if joystick:
        acciones.append(Node(package='joy', executable='joy_node',
                             parameters=[{'autorepeat_rate': 20.0}]))
    return acciones


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ev3_ip', default_value='192.168.1.103'),
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('usar_mallas', default_value='true'),
        DeclareLaunchArgument('balines_delanteros', default_value='false'),
        OpaqueFunction(function=configurar),
    ])
