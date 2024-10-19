cmake_minimum_required(VERSION 3.8)
project(robit_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/VisionMsg.msg"
  "msg/MasterMsg.msg"
  "msg/SimpleMoveMsg.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()