cmake_minimum_required(VERSION 3.5)
project(openni2_camera_param_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetSerial.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(
  builtin_interfaces
  rosidl_default_runtime
)
ament_package()

