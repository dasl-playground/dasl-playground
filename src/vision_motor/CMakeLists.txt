cmake_minimum_required(VERSION 3.5)
project(vision_motor)

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
find_package(geometry_msgs REQUIRED)

message("${vision_motor_SOURCE_DIR}")
message("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(
 # /home/dasl-vision/src/DaslPanMotionController/src 
  ${CMAKE_CURRENT_SOURCE_DIR}/../dasl_playground
 # /opt/ros/dashing/include/
)

add_executable(visionmotorposinput src/publisher.cpp)
ament_target_dependencies(visionmotorposinput rclcpp std_msgs geometry_msgs)
# target_include_directories(visionmotorposinput
#   PUBLIC
#     $<BUILD_INTERFACE:/opt/ros/dashing/include/>
#     $<INSTALL_INTERFACE:include>)

add_executable(VisionMotor src/subscriber.cpp )
ament_target_dependencies(VisionMotor rclcpp st_msgs geometry_msgs)

#target_include_directories(VisionMotor
#  PUBLIC
#     $<BUILD_INTERFACE:/opt/ros/dashing/include/:/home/dasl-vision/src/dasl_playground/>
#     $<INSTALL_INTERFACE:include>)

add_executable(DaslPanMotionController src/main.cpp  )

install(TARGETS
  visionmotorposinput
  VisionMotor
  DaslPanMotionController
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
