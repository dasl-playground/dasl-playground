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
//Title : ROS2 Vision motor position subscriber.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <daslpg/daslpg>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

using std::placeholders::_1;

struct termios orig_termios;
auto &&ctrlPan = DaslPanMotionController::getInstance();

void reset_terminal_mode()
{
  tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
  struct termios new_termios;

  /* take two copies - one for now, one for later */
  tcgetattr(0, &orig_termios);
  memcpy(&new_termios, &orig_termios, sizeof(new_termios));

  /* register cleanup handler, and set the new terminal mode */
  atexit(reset_terminal_mode);
  cfmakeraw(&new_termios);
  tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
  struct timeval tv = {0L, 0L};
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
  int r;
  unsigned char c;
  if ((r = read(0, &c, sizeof(c))) < 0)
  {
    return r;
  }
  else
  {
    return c;
  }
}

class VisionMotorPosSub : public rclcpp::Node
{
public:
  VisionMotorPosSub()
      : Node("VisionMotorPoseSub")
  {
    subscription = this->create_subscription<geometry_msgs::msg::Point>(
        "motorpose", 100, std::bind(&VisionMotorPosSub::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) const
  {
    
    int scanpose = (int)msg->x;
    ctrlPan->setPosition(scanpose, 5);
    float crntpose = ctrlPan->getPosition();

    printf("Target position %d, Current Position %f \n", scanpose, crntpose);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription;
};

int main(int argc, char *argv[])
{

  int ret = ctrlPan->connect();
  if (ret == 0)
  {
    printf("Homing\n");
    ctrlPan->findHome();

    printf("After finishing finding home position, press any key\n");
    getch();

    ctrlPan->setPosition(0, 5);
    printf("A setPosition(0)\n");
    printf("After finishing to approach set position, press any key for initial scanning\n");
    getch();

    ctrlPan->scan(-50, 50, 5, 2);
    printf("When initial scanning is done, press any key for random positioning\n");
    getch();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionMotorPosSub>());
    rclcpp::shutdown();
  }

  ctrlPan->disconnect();
  return 0;
}
