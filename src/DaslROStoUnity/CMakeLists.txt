cmake_minimum_required(VERSION 3.5)
project(file_server)

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

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dasl_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

#find_package(catkin  REQUIRED COMPONENTS
#  roscpp
#  rospy
#  std_msgs
#  message_generation
#  roslib
#)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
#rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/GetBinaryFile.srv"
#        "srv/SaveBinaryFile.srv"
#        DEPENDENCIES std_msgs
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)


#catkin_package(
##  INCLUDE_DIRS include
#  LIBRARIES file_services
#  CATKIN_DEPENDS roscpp rospy std_msgs roslib
#  DEPENDS message_runtime
#)
#ament_package(
#        #  INCLUDE_DIRS include
#        #ament_export_libraries file_services
#        ament_export_dependencies rclcpp std_msgs dasl_interface
##        DEPENDS message_runtime
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(file_server src/file_server.cpp)
ament_target_dependencies(file_server
        "rclcpp"
        "rclcpp_action"
        "std_msgs"
        "dasl_interface"
        "ament_index_cpp"
        )
#target_link_libraries(file_server ${catkin_LIBRARIES})
#add_dependencies(file_server file_server_gencpp)

install(TARGETS file_server
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY launch param urdf
        DESTINATION share/${PROJECT_NAME})

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