#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <sstream>

#include <trikControl/brickFactory.h>
#include <trikControl/brickInterface.h>
#include <trikControl/ledInterface.h>

#include <QtGui/QApplication>

int qtargc = 2;
char *qtargv[] = {"trikroscpp_node", "-qws"};

trikControl::BrickInterface *brick;
const int queue_length = 1000;

class Handle {
public:
  Handle(trikControl::DeviceInterface * const device,
	 const std::string& port) :
    device_(device),
    port_(port)
  {}

protected:
  trikControl::DeviceInterface * const device_;
  const std::string port_;
};

class Publisher {
public:
  virtual void publish() const = 0;
protected:
  ros::Publisher pub_;
};

class Subscriber {
protected:
  virtual void handle() = 0;
  ros::Subscriber sub_;
};

class MotorHandle : public Handle, public Subscriber {
public:
  // need to change this
  MotorHandle(trikControl::MotorInterface * const device,
	      const std::string& port) :
    Handle(device, port),
    motor_(device)
  {}
  
  void init(ros::NodeHandle& nh) { 
    std::stringstream name;
    name << "motor_" << this->port_;
    
    this->sub_ = nh.subscribe(name.str(), queue_length, 
			      &MotorHandle::handle, this);
  }

  void handle(const std_msgs::Int32::ConstPtr& msg) {
    this->motor_->setPower(msg->data);
  }

private:
  trikControl::MotorInterface * const motor_;
};

class SensorHandle : public Handle, public Publisher {
public:
  SensorHandle(trikControl::SensorInterface * const device,
              const std::string& port) :
    Handle(device, port),
    sensor_(device)
  {}

  void init(ros::NodeHandle& nh) {
    std::stringstream name;
    name << "sensor_" << this->port_;

    this->pub_ = nh.advertise<std_msgs::Int32>(name.str(), queue_length);
  }

  void publish() const {
    std_msgs::Int32 msg;
    msg.data = sensor_->read();
    this->pub_.publish(msg);
  }

private:
  trikControl::SensorInterface * const sensor_;
};

void ledCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  trikControl::LedInterface *led = brick->led();

  if (msg->data == "orange") {
    led->orange();
  } else if (msg->data == "red") {
    led->red();
  } else if (msg->data == "green") {
    led->green();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trikroscpp_node");

  QApplication app(qtargc, qtargv);
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  const int queue_length = 1000;
  ros::Subscriber sub = n.subscribe("trikLed", queue_length, ledCallback);

  brick = trikControl::BrickFactory::create(".", ".");

  std::vector<SensorHandle> vsh;
  // init 
  const QStringList& sensors(brick->sensorPorts(trikControl::SensorInterface::Type::analogSensor));
  for (QStringList::const_iterator it = sensors.begin(); it != sensors.end(); ++it) {
    ROS_INFO("SENSOR: [%s]", it->toStdString().c_str());
    trikControl::SensorInterface *sns = brick->sensor(*it);
    if (sns->status() == trikControl::DeviceInterface::Status::ready) {
      ROS_INFO("SENSOR is ready: [%s]", it->toStdString().c_str());
      SensorHandle sh(sns, it->toStdString());
      sh.init(n);
      vsh.push_back(sh);
    }
  }
  
  sensors = brick->sensorPorts(trikControl::SensorInterface::Type::digitalSensor);
  for (QStringList::const_iterator it = sensors.begin(); it != sensors.end(); ++it) {
    ROS_INFO("SENSOR: [%s]", it->toStdString().c_str());
    trikControl::SensorInterface *sns = brick->sensor(*it);
    if (sns->status() == trikControl::DeviceInterface::Status::ready) {
      ROS_INFO("SENSOR is ready: [%s]", it->toStdString().c_str());
      SensorHandle sh(sns, it->toStdString());
      sh.init(n);
      vsh.push_back(sh);
    }
  }

  while (ros::ok())
  {
    for (const SensorHandle& sh : vsh) {
      sh.publish();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
