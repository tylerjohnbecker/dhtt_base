cmake_minimum_required(VERSION 3.5)
project(dhtt_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-undef)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dhtt REQUIRED)
find_package(dhtt_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(dhtt_plugins SHARED src/dhtt_plugins.cpp
                                src/tasks/root_behavior.cc src/tasks/and_behavior.cc src/tasks/then_behavior.cc src/tasks/or_behavior.cc
                                src/behaviors/test/test_behavior.cc src/behaviors/test/mock_move_behavior.cc src/behaviors/test/mock_look_behavior.cc
                                src/behaviors/action_type.cc src/behaviors/move_behavior.cc src/behaviors/pick_behavior.cc src/behaviors/place_behavior.cc
                                src/behaviors/cooking_execute_behavior.cc src/behaviors/cooking_interact_special_behavior.cc src/behaviors/cooking_move_behavior.cc
                                src/behaviors/cooking_pick_behavior.cc src/behaviors/cooking_place_behavior.cc
                                src/goitrs/goitr_test.cc src/goitrs/find_goitr.cc)

target_include_directories(dhtt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dhtt_plugins dhtt::dhtt_lib)
ament_target_dependencies( dhtt_plugins
  dhtt
  dhtt_msgs
  pluginlib
  rclcpp
  rclcpp_action
)

pluginlib_export_plugin_description_file(dhtt plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(dhtt_plugins PRIVATE "DHTT_PLUGINS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS dhtt_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_include_directories(
#   include
# )
ament_export_libraries(
  dhtt_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
