#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <trikControl/brickFactory.h>
#include <trikControl/brickInterface.h>
#include <trikControl/ledInterface.h>
#include <QtGui/QApplication>
int qtargc = 2;
char * qtargv[] = {"trikroscpp_node", "-qws"};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trikroscpp_node");

  QApplication app(qtargc, qtargv);
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(2);

  ROS_INFO("%p\n", app.thread());

  trikControl::BrickInterface *brick = trikControl::BrickFactory::create(".", ".");

  int count = 0;
  trikControl::LedInterface *led = brick->led();

  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    switch (count % 3) {
    case 0: 
      led->orange();
      break;
    case 1:
      led->green();
      break;
    case 2:
      led->red();
      break;
    }

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
