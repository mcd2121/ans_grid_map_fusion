cmake_minimum_required(VERSION 3.8)
project(ans_grid_map_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # Replace with actual grid map message dependencies
find_package(pluginlib REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)

include_directories(
    include
  )

set(dependencies
  rclcpp
  std_msgs
  pluginlib
  grid_map 
  grid_map_core 
  grid_map_ros 
  grid_map_msgs 

)

add_library(fusion_plugins src/EvidentialFusion.cpp)
ament_target_dependencies(fusion_plugins SYSTEM
    ${dependencies}
)

add_executable(global_fusion_node src/global_fusion_node.cpp)
ament_target_dependencies(global_fusion_node SYSTEM
    ${dependencies}
)
# Install targets
install(TARGETS
  global_fusion_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Mark other files for installation
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
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
pluginlib_export_plugin_description_file(ans_grid_map_fusion_base fusion_plugins.xml)
ament_package()
