cmake_minimum_required(VERSION 3.16...3.28)
project(ufo_mapping)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  ufo_msgs
  ufo_ros
  visualization_msgs
  tf2_sensor_msgs
)

find_package(UFO REQUIRED 
  COMPONENTS 
    Map 
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES ufo_mapping_server_nodelet
  CATKIN_DEPENDS roscpp ufo_msgs ufo_ros
# DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server_node 
  src/server_node.cpp
  src/server.cpp
)

add_dependencies(${PROJECT_NAME}_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_server_node
  ${catkin_LIBRARIES}
  UFO::Map
)

install(TARGETS ${PROJECT_NAME}_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
