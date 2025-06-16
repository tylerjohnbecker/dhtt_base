cmake_minimum_required(VERSION 3.5)
project(dhtt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20 ( for reasons )
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  # set(CMAKE_CXX_COMPILER g++-10)
  # set(CMAKE_CXX_FLAGS "-static-libstdc++")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dhtt_msgs REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(pluginlib REQUIRED)

find_package(yaml-cpp REQUIRED)

find_library(LIBSTDCXX_10 libstdc++.so.6 PATHS)

# make nodes
add_library(dhtt_lib SHARED src/utils.cc src/tree/node.cc src/server/main_server.cc src/planning/goitr_type.cc src/server/sub_server.cc)
target_include_directories(dhtt_lib PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dhtt_lib rclcpp rclcpp_action dhtt_msgs pluginlib)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS dhtt_lib
  EXPORT dhtt_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(start_server src/main.cpp )
ament_target_dependencies(start_server rclcpp rclcpp_action rcl_yaml_param_parser dhtt_msgs yaml-cpp pluginlib)
target_include_directories(start_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(start_server dhtt_lib ${YAML_CPP_LIBRARIES})

add_executable(param_node src/param_node.cpp)
ament_target_dependencies(param_node rclcpp)

install(TARGETS start_server param_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/experiment_1.py
  scripts/experiment_2.py
  scripts/experiment_3.py
  scripts/visualize_results.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    tests/test_add_remove.py
    tests/test_behavior_order.py
    tests/test_dynamic_changes.py
    tests/test_pre_post.py
    # tests/test_goitr_basic.py
    )

  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

  # install(PROGRAMS
  #   tests/add_remove_test.py
  #   DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(dhtt_lib)
ament_export_targets(dhtt_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action dhtt_msgs pluginlib)

ament_package()
