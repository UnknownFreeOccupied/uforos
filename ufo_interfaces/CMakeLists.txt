cmake_minimum_required(VERSION 3.8)
project(ufo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Map.msg"
  "msg/MapInfo.msg"
  "msg/MapTypeInfo.msg"
  "msg/NavMap.msg"
  "msg/Version.msg"
)
set(srv_files
  "srv/ClearMap.srv"
  "srv/GetMap.srv"
  "srv/SaveMap.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
