cmake_minimum_required(VERSION 3.5)
project(dhtt)

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
find_package(rclcpp REQUIRED)
find_package(dhtt_msgs REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  thread)

# make nodes
add_executable(start_server src/main.cpp src/server/main_server.cc)
ament_target_dependencies(start_server rclcpp rcl_yaml_param_parser dhtt_msgs yaml-cpp)
target_link_libraries(start_server ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(start_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS start_server
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    tests/test_add_remove.py
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

ament_package()
