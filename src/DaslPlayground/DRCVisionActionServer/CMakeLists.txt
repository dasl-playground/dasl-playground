cmake_minimum_required(VERSION 3.5)
project(drc_vision_action_server)

set(DASL_PLAYGROUND dasl_playground)
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
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dasl_action REQUIRED)




add_executable(drc_vision_action_server DRCVisionActionServer.cpp)
ament_target_dependencies(drc_vision_action_server
        "rclcpp"
        "rclcpp_action"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "dasl_action"
)

# #Library for LiDar3D
target_link_libraries(drc_vision_action_server urg_cpp)

install(TARGETS drc_vision_action_server
  DESTINATION lib/${PROJECT_NAME}
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
