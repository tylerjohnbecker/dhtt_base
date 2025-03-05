cmake_minimum_required(VERSION 3.5)
project(dhtt_msgs)

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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # message files
  "msg/Node.msg"
  "msg/Subtree.msg"
  "msg/Resource.msg"
  "msg/Resources.msg"
  "msg/NodeStatus.msg"
  "msg/CookingAction.msg"
  "msg/CookingObject.msg"
  "msg/CookingObservation.msg"
  "msg/CookingRecipe.msg"
  "msg/CookingAgent.msg"

  # service files
  "srv/ControlRequest.srv"
  "srv/FetchRequest.srv"
  "srv/ModifyRequest.srv"
  "srv/HistoryRequest.srv"
  "srv/InternalControlRequest.srv"
  "srv/InternalModifyRequest.srv"
  "srv/InternalServiceRegistration.srv"
#  "srv/NutreeJsonRequest.srv"
  "srv/CookingRequest.srv"

  # action files
  "action/Activation.action"

  # dependecies for messages
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
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

ament_package()
