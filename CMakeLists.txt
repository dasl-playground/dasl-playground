cmake_minimum_required(VERSION 3.16)
project(dev_ws)

set(CMAKE_CXX_STANDARD 14)

include_directories(src/DaslLibrary)
include_directories(src/DaslLibrary/include)
include_directories(src/DaslLibrary/include/DaslLibrary)
include_directories(src/DaslPlayground/DaslDRCVision/include)
include_directories(/opt/ros/foxy/include)

add_executable(dev_ws
        src/DaslLibrary/include/DaslLibrary/DaslPanMotionController.h
        src/DaslLibrary/include/DaslLibrary/DaslRS232.h
        src/DaslPlayground/DaslDRCVision/include/DaslDRCVisionNode.h
        src/DaslPlayground/DaslDRCVision/include/DaslLidarActionServerNode.h
        src/DaslPlayground/DaslDRCVision/src/DaslDRCVisionNode.cpp
        src/DaslPlayground/DaslDRCVision/src/main.cpp
        src/DaslPlayground/DaslDRCVision/src/subscriber.cpp)
