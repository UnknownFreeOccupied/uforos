cmake_minimum_required(VERSION 3.8.2)
project(ufo_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  ufo_msgs
)

find_package(UFO REQUIRED 
  COMPONENTS 
    Cloud
    Math
    Vision
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES}
  CATKIN_DEPENDS geometry_msgs sensor_msgs ufo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
  src/conversions.cpp
)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${catkin_LIBRARIES}
    UFO::Cloud
    UFO::Math
    UFO::Vision
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)