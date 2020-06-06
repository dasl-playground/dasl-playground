//Copyright (c) 2020 Dongbin Kim, Giho Jang
//
// Permission is hereby granted, free of charge,
// to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software
// without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall
// be included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
//  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
//  AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
//  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#ifndef DASL_LIDAR_ACTION_NODE_H
#define DASL_LIDAR_ACTION_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <DaslLibrary/DaslLibrary>
#include <chrono>


#include "rclcpp_action/rclcpp_action.hpp"
//#include "dasl_playground/action/dasl_lidar_action.hpp"
/*
class DaslLidarActionServerNode : public rclcpp::Node
{
  
  rclcpp_action::Server<DaslLidarAction>::SharedPtr action_server;
  

  qrk::Urg_driver mLidar;
  DaslPanMotionController *mMotion;

  std::vector<long> mRawData;
  long mLidarTimeStamp;

public:
  DaslLidarActionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : 
    mMotion(DaslPanMotionController::getInstance()),
    Node("VisionMotorPosePublisher", options)
  {
    mLidarTimeStamp = 0;
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    if (!init())
    {
      RCLCPP_ERROR(get_logger(), "Device Error");
      exit(0);
    }
    RCLCPP_INFO(get_logger(), "Successed to open devices");
    // mPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("motorpose1", 100);
    // mTimer = create_wall_timer(
    //     25ms, std::bind(&DaslLidarActionServerNode::pub_topic, this));
    // addSubscription();
  
  }

  ~DaslLidarActionServerNode()
  {
    RCLCPP_INFO(get_logger(), "Release Devices");
    release();
  }

private:
  bool init(std::string ip_address = "10.19.3.10")
  {

    //RCLCPP_INFO(get_logger(), "mLidar.is_open()");
    if (mLidar.is_open())
    {
      return true;
    }
    //RCLCPP_INFO(get_logger(), "mMotion->connect()");
    mMotion->connect();

    //RCLCPP_INFO(get_logger(), "mLidar.open()");
    if (!mLidar.open(ip_address.c_str(), qrk::Urg_driver::Default_port, qrk::Urg_driver::Ethernet))
    {
      return false;
    }
    mLidar.stop_measurement();
    mLidar.start_measurement();
    return true;
  }
  bool release()
  {
    mLidar.close();
    mMotion->disconnect();
    return true;
  }

  void pub_topic()
  {
    auto message = sensor_msgs::msg::PointCloud2();

    if (!mLidar.get_distance(mRawData, &mLidarTimeStamp))
    {
      RCLCPP_INFO(get_logger(), "Urg_driver::get_distance(): %s", mLidar.what());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Success!! %d", mRawData[0]);
    }
    mPublisher->publish(message);
  }

  void addSubscription()
  {
    //  mSubscription = create_subscription<std_msgs::msg::String>( 
    //    "motorpose2", //name of topic
    //    1, //number of topic histories
    //  [&](const std_msgs::msg::String::SharedPtr msg){

      
    //     RCLCPP_INFO(this->get_logger(), "I heard mode: '%s'", msg->data.c_str());
    //  });
  }

};
*/
#endif
