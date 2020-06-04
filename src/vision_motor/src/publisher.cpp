//Copyright (c) 2020 Dongbin Kim, Dr. GiHo Jang
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

//Author : Dongbin Kim, GiHo Jang
//Date : 2020-06-04
//Title : ROS2 Vision motor position publisher

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <thread>



double motorpose;

void motorposeinput()
{
  while (rclcpp::ok())
  {
    printf("type position of motor \n");
    scanf("%lf", &motorpose);
  }
}
class VisionMotorPosPub : public rclcpp::Node
{
public:
  VisionMotorPosPub()
      : Node("VisionMotorPosePublisher")
  {
    using namespace std::chrono_literals;
    publisher = this->create_publisher<geometry_msgs::msg::Point>("motorpose", 100);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&VisionMotorPosPub::timer_callback, this));
  }

private:
  void timer_callback()
  {

    auto message = geometry_msgs::msg::Point();
    message.x = motorpose;
    publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::thread t1(motorposeinput);
  rclcpp::spin(std::make_shared<VisionMotorPosPub>());
  rclcpp::shutdown();
  return 0;
}
