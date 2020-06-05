//Copyright (c) 2020 Dongbin Kim, Dr. Giho Jang
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

//Author : Dongbin Kim, Giho Jang
//Date : 2020-06-04
//Title : ROS2 Vision motor position publisher

#include <chrono>
#include <memory>
#include <thread>
#include <daslpg/daslpg>

double motorpose;

// void motorposeinput()
// {
//   while (rclcpp::ok())
//   {
//     printf("type position of motor \n");
//     scanf("%s", &motorpose);
//   }
// }


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // std::thread t1(motorposeinput);
  rclcpp::spin(std::make_shared<DaslLidarActionNode>());
  rclcpp::shutdown();
  return 0;
}
