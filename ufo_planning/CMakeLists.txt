cmake_minimum_required(VERSION 3.8)
project(ufo_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(UFO REQUIRED COMPONENTS Plan)
find_package(ufo_interfaces REQUIRED)

foreach(DIM IN ITEMS 2 3)
  add_executable(server_${DIM}d src/server_node.cpp)

  target_compile_definitions(server_${DIM}d PRIVATE DIMENSIONS=${DIM})

  ament_target_dependencies(server_${DIM}d
    nav_msgs
    rclcpp
    ufo_interfaces
  )

  target_link_libraries(server_${DIM}d
    UFO::Plan
  )

  target_include_directories(server_${DIM}d PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  install(TARGETS
    server_${DIM}d
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
