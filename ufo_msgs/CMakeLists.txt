cmake_minimum_required(VERSION 3.16...3.28)
project(ufo_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

find_package(UFO REQUIRED 
  COMPONENTS 
    Map 
)

add_message_files(
  FILES
  Map.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS ufomap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME} STATIC
#   src/conversions.cpp
# )

# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} UFO::Map ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}
#   PUBLIC
#     ${catkin_LIBRARIES}
#     UFO::Map
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
# )