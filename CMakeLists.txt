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
find_package(builtin_interfaces REQUIRED)

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
  builtin_interfaces
)

set(fusion_libs
  evidential_fusion_plugin
  )
add_library(evidential_fusion_plugin SHARED src/EvidentialFusion.cpp)

foreach(lib_name ${fusion_libs})
  ament_target_dependencies(${lib_name} SYSTEM
    ${dependencies}
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(${lib_name} PRIVATE "SHELL:--param ggc-min-expand=1")
    target_compile_options(${lib_name} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
  endif()
endforeach()



add_executable(global_fusion_node src/global_fusion_node.cpp)
ament_target_dependencies(global_fusion_node SYSTEM
    ${dependencies}
)

# Install targets
install(TARGETS
  ${fusion_libs}
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
install(
  FILES fusion_plugins.xml
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
ament_export_dependencies(${dependencies})
ament_export_libraries(${fusion_libs})
pluginlib_export_plugin_description_file(ans_grid_map_fusion fusion_plugins.xml)
ament_package()
