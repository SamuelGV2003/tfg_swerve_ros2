cmake_minimum_required(VERSION 3.8)
project(swerve_robot_description)

find_package(ament_cmake REQUIRED)

# Paquete sin código compilado: solo se instalan los archivos que usan
# los launch (modelo, configuración, lanzadores y mallas).
install(
  DIRECTORY urdf config launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()