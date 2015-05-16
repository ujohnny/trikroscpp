#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <sstream>

#include <trikControl/brickFactory.h>
#include <trikControl/brickInterface.h>
#include <trikControl/ledInterface.h>

#include <QtGui/QApplication>

int qtargc = 2;
char *qtargv[] = {"trik_ros_node", "-qws"};

trikControl::BrickInterface *brick;
const int queue_length = 1000;

template <class T>
class Handle 
{
public:
  Handle(T * const device,
	 const std::string& port) : 
    device_(device) 
  {}

protected:
  T *device_;
};

class Publisher 
{
public:
  virtual void publish() const = 0;

protected:
  ros::Publisher pub_;
};

template <class T>
class Subscriber 
{
protected:
  ros::Subscriber sub_;
  virtual void handle(typename T::ConstPtr& msg) = 0;
};

class MotorHandle : public Handle<trikControl::MotorInterface>, 
		    public Subscriber<std_msgs::Int32> 
{
public:
  // need to change this
  MotorHandle(trikControl::MotorInterface * const device,
	      const std::string& port)
  {
    std::string name(prefix);
    std::transform(port.begin(), port.end(), std::back_inserter(name), ::tolower);
    
    this->sub_ = nh.subscribe(name.str(), queue_length, 
			      &MotorHandle::handle, this);
  }

private:  
  static const std::string prefix;
  void handle(const std_msgs::Int32::ConstPtr& msg) 
  {
    this->device_->setPower(msg->data);
  }
};

const std::string MotorHandle::prefix = "motor_";

class SensorHandle : public Handle<trikControl::SensorInterface>, 
		     public Publisher 
{
public:
  SensorHandle(trikControl::SensorInterface * const device,
	       const std::string& port) 
  {
    std::string name(prefix);
    std::transform(port.begin(), port.end(), std::back_inserter(name), ::tolower);
   
    this->pub_ = nh.advertise<std_msgs::Int32>(name.str(), queue_length);
  }

  void publish() const {
    std_msgs::Int32 msg;
    msg.data = sensor_->read();
    this->pub_.publish(msg);
  }

private:
  static const std::string prefix;
};

const std::string SensorHanlde::prefix =  "sensor_";

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
  ros::init(argc, argv, "trik_ros_node");

  QApplication app(qtargc, qtargv);
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  const int queue_length = 1000;
  ros::Subscriber sub = n.subscribe("trikLed", queue_length, ledCallback);

  brick = trikControl::BrickFactory::create(".", ".");

  std::vector<SensorHandle> vsh;
  // init 
  const QStringList& asensors(brick->sensorPorts(trikControl::SensorInterface::Type::analogSensor));
  for (QStringList::const_iterator it = asensors.begin(); it != asensors.end(); ++it) {
    ROS_INFO("SENSOR: [%s]", it->toStdString().c_str());
    trikControl::SensorInterface *sns = brick->sensor(*it);
    if (sns->status() == trikControl::DeviceInterface::Status::ready) {
      ROS_INFO("SENSOR is ready: [%s]", it->toStdString().c_str());
      SensorHandle sh(sns, it->toStdString());
      vsh.push_back(sh);
    }
  }

  /*  
  const QStringList& dsensors = brick->sensorPorts(trikControl::SensorInterface::Type::digitalSensor);
  for (QStringList::const_iterator it = dsensors.begin(); it != dsensors.end(); ++it) {
    ROS_INFO("SENSOR: [%s]", it->toStdString().c_str());
    trikControl::SensorInterface *sns = brick->sensor(*it);
    if (sns->status() == trikControl::DeviceInterface::Status::ready) {
      ROS_INFO("SENSOR is ready: [%s]", it->toStdString().c_str());
      SensorHandle sh(sns);
      sh.init(n, it->toStdString());
      vsh.push_back(sh);
    }
  }

  std::vector<MotorHandle *> vm;
  const QStringList& pmotors = brick->motorPorts(trikControl::MotorInterface::Type::powerMotor);
  for (QStringList::const_iterator it = pmotors.begin(); it != pmotors.end(); ++it) {
    ROS_INFO("MOTOR: [%s]", it->toStdString().c_str());
    trikControl::MotorInterface *mi = brick->motor(*it);
    if (mi->status() == trikControl::DeviceInterface::Status::ready) {
      ROS_INFO("MOTOR is ready: [%s]", it->toStdString().c_str());
      MotorHandle *mh = new MotorHandle(mi);
      mh->init(n, it->toStdString());
      vm.push_back(mh);
    }
  }

  const QStringList& smotors = brick->motorPorts(trikControl::MotorInterface::Type::servoMotor);
  for (QStringList::const_iterator it = smotors.begin(); it != smotors.end(); ++it) {
    ROS_INFO("MOTOR: [%s]", it->toStdString().c_str());
    trikControl::MotorInterface *mi = brick->motor(*it);
    if (mi->status() == trikControl::DeviceInterface::Status::ready) {
      ROS_INFO("MOTOR is ready: [%s]", it->toStdString().c_str());
      MotorHandle *mh = new MotorHandle(mi);
      mh->init(n, it->toStdString());
      vm.push_back(mh);
    }
  }
  */
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
