cmake_minimum_required(VERSION 3.8)
project(robo_driver_dll)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msg_srv REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(high_level_node src/hld/main.cpp src/hld/highLevelNode.cpp src/hld/servoUtils.cpp src/hld/context.cpp src/hld/state.cpp src/hld/idleState.cpp src/hld/movingState src/hld/emergencyStopState)
ament_target_dependencies(high_level_node rclcpp msg_srv)    # CHANGE

add_executable(cli_communication src/cli/main.cpp src/cli/commandParser.cpp src/cli/communicatorNode.cpp)
ament_target_dependencies(cli_communication rclcpp msg_srv)

add_library(low_level_lib src/lld/command.cpp src/lld/lowLevelServer.cpp src/lld/commandUtils.cpp src/lld/multiServoCommand.cpp src/lld/singleServoCommand.cpp)
ament_target_dependencies(low_level_lib rclcpp msg_srv)    # CHANGE

target_link_libraries(high_level_node low_level_lib)

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

install(TARGETS
  high_level_node
  cli_communication
  low_level_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
