cmake_minimum_required(VERSION 3.8)
project(dhtt_reorder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dhtt REQUIRED)
find_package(dhtt_msgs REQUIRED)
find_package(dhtt_plugins REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(dhtt_plot REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_library(dhtt_reorder SHARED src/dhtt_reorder.cpp)
add_executable(experiment_moveit src/experiment_moveit.cpp)
target_include_directories(dhtt_reorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(experiment_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dhtt_reorder dhtt::dhtt_lib)
target_compile_features(experiment_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies( dhtt_reorder
  dhtt
  dhtt_msgs
  dhtt_plugins
  pluginlib
  rclcpp
  rclcpp_action
)
ament_target_dependencies( experiment_moveit
  moveit_ros_planning_interface
  rclcpp
  dhtt_msgs
)

pluginlib_export_plugin_description_file(dhtt plugins.xml)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS dhtt_reorder
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS experiment_moveit
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  dhtt_reorder/dHTT_reorder.py
  DESTINATION lib/${PROJECT_NAME})

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

  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_reorder_dHTT.py
    test/test_reorder.py
    # Add other test files here
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(
  dhtt_reorder
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
