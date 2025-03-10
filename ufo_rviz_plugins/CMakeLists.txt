cmake_minimum_required(VERSION 3.8)
project(ufo_rviz_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(UFO REQUIRED COMPONENTS Map Plan)
find_package(ufo_interfaces REQUIRED)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MAP_MOC_FILES
  include/ufo_rviz_plugins/map_display.hpp
)

add_library(map_display src/map_display.cpp ${MAP_MOC_FILES})

target_include_directories(map_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(map_display
  pluginlib
  rclcpp
  rviz_common
  ufo_interfaces
)

target_link_libraries(map_display
  UFO::Map
)

qt5_wrap_cpp(NAV_MAP_MOC_FILES
  include/ufo_rviz_plugins/nav_map_display.hpp
)

add_library(nav_map_display src/nav_map_display.cpp ${NAV_MAP_MOC_FILES})

target_include_directories(nav_map_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav_map_display
  pluginlib
  rclcpp
  rviz_common
  ufo_interfaces
)

target_link_libraries(nav_map_display
  UFO::Plan
)

install(TARGETS map_display nav_map_display
  EXPORT export_ufo_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES ufo_rviz_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_targets(export_ufo_rviz_plugins)

pluginlib_export_plugin_description_file(rviz_common ufo_rviz_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
