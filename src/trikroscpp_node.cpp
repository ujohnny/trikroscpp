#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <trikControl/brick.h>
#include <QtGui/QApplication>
#include <QtCore/QDir>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trikroscpp_node");

  QApplication app(argc, argv);
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(2);

  ROS_INFO("%p\n", app.thread());
  //  ROS_INFO("%s\n", QDir::currentPath());
  trikControl::Brick *b = new trikControl::Brick(*app.thread()
						 , QDir::currentPath() + '/', QDir::currentPath() + '/');
  int count = 0;
  trikControl::Led *led = b->led();

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
