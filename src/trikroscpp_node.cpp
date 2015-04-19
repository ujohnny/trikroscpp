#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <trikControl/brick.h>
#include <QtWidgets/QApplication>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trikroscpp_node");

  QApplication app(argc, argv);
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(2);

  ROS_INFO("%p\n", app.thread());
  trikControl::Brick *b = new Brick(*app.thread()
				    , "./system-config.xml", "./");

  trikControl::Led *l = b->led();

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
