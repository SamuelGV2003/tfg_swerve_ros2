# tfg_swerve_ros2

Modelado, simulación y control de una plataforma móvil swerve drive en entorno ROS 2.

Código del Trabajo Final de Graduación de Samuel Gómez Villanueva (Ingeniería Mecatrónica, Instituto Tecnológico de Costa Rica), realizado en la Universitat Politècnica de València en 2026.

La plataforma tiene dos módulos swerve, cada uno con un motor de dirección y uno de tracción, y apoyos pasivos. El repositorio incluye el modelo cinemático, el modelo digital en Gazebo con ROS 2, el controlador de punto objetivo, los scripts para analizar las pruebas y los programas del prototipo a escala.

## Contenido

```
matlab/      Cinemática inversa y directa, y modelo del controlador de punto objetivo
ros2/        Paquetes de ROS 2
  swerve_robot_description/   Descripción del robot (URDF/xacro), mallas, controladores y archivos de lanzamiento
  swerve_control/             Nodo de cinemática, nodo de punto objetivo y puente de la sombra digital
analisis/    Scripts de Python para analizar las corridas en Gazebo
blender/     Script que procesa las mallas exportadas de BrickLink Studio
prototipo/   Programas en C del prototipo a escala
```

## Requisitos

- Ubuntu 24.04 con ROS 2 Jazzy y Gazebo Harmonic
- ros_gz_sim, ros_gz_bridge, gz_ros2_control y ros2_controllers
- Para la sombra digital, además joy y un controlador de consola
- Python 3 con numpy y matplotlib para los scripts de análisis
- MATLAB para los scripts de la carpeta matlab
- Blender 4.5 para el script de mallas
- Biblioteca ev3dev-c para compilar los programas del prototipo

## Compilar los paquetes

Copiar las dos carpetas de `ros2/` dentro de la carpeta `src` de un workspace de ROS 2 y compilar:

```
cd ~/ROS_TFG
colcon build
source install/setup.bash
```

## Uso

Los archivos de lanzamiento están en la carpeta `launch` de cada paquete y explican en su encabezado cómo se usan. Los scripts de `analisis/` y los programas de `prototipo/` también. El procedimiento completo está en el apéndice del trabajo escrito.

## Videos

Los videos de las pruebas en simulación y en el prototipo están en la sección Releases del repositorio.
