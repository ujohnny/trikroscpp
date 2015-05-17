#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include <memory>
#include <vector>
#include <algorithm>

#include <trikControl/brickFactory.h>
#include <trikControl/brickInterface.h>

#include <QtGui/QApplication>

int qtargc = 2;
char *qtargv[] = {"trik_ros_node", "-qws"};

trikControl::BrickInterface *brick;
const int queue_length = 1000;

template <class T>
class Handle 
{
public:
  Handle(T * const device) : 
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
	      const std::string& port,
	      ros::NodeHandle& nh) :
    Handle(device)
  {
    std::string name(prefix);
    std::transform(port.begin(), port.end(), std::back_inserter(name), ::tolower);
    
    this->sub_ = nh.subscribe(name, queue_length, 
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
	       const std::string& port,
	       ros::NodeHandle& nh) :
    Handle(device)
  {
    std::string name(prefix);
    std::transform(port.begin(), port.end(), std::back_inserter(name), ::tolower);
   
    this->pub_ = nh.advertise<std_msgs::Int32>(name, queue_length);
  }

  void publish() const {
    std_msgs::Int32 msg;
    msg.data = device_->read();
    this->pub_.publish(msg);
  }

private:
  static const std::string prefix;
};

const std::string SensorHandle::prefix =  "sensor_";

class VectorSensorHandle : public Handle<trikControl::VectorSensorInterface>, 
			   public Publisher 
{
public:
  VectorSensorHandle(trikControl::VectorSensorInterface * const device,
		     const std::string& port,
		     ros::NodeHandle& nh) :
    Handle(device)
  {
    std::string name(prefix);
    std::transform(port.begin(), port.end(), std::back_inserter(name), ::tolower);
   
    this->pub_ = nh.advertise<geometry_msgs::Vector3>(name, queue_length);
  }

  void publish() const {
    geometry_msgs::Vector3 msg;
    QVector<int> v = device_->read();
    msg.x = v[0]; 
    msg.y = v[1];
    msg.z = v[2];
    this->pub_.publish(msg);
  }

private:
  static const std::string prefix;
};

const std::string VectorSensorHandle::prefix =  "vsensor_";

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
  ros::Rate loop_rate(100); // 100 Hz

  const int queue_length = 1000;
  ros::Subscriber sub = n.subscribe("trikLed", queue_length, ledCallback);

  brick = trikControl::BrickFactory::create(".", ".");

  std::vector< std::shared_ptr<Publisher> > vsh;

  // init 
  auto initSensors = [&n] (const QString& port) {
    ROS_INFO("SENSOR: [%s]", port.toStdString().c_str());
    trikControl::SensorInterface *sns = brick->sensor(port);
    if (sns->status() == trikControl::DeviceInterface::Status::ready) {
      ROS_INFO("SENSOR is ready: [%s]", port.toStdString().c_str());
      return std::make_shared<SensorHandle>(sns, port.toStdString(), n);
    } else {
      // temp fix
      return std::shared_ptr<SensorHandle>();
    }
  };

  std::list<QString> asensors(brick->sensorPorts(trikControl::SensorInterface::Type::analogSensor).toStdList());
  std::transform(asensors.begin(), asensors.end(), std::back_inserter(vsh), initSensors);

  std::list<QString> dsensors(brick->sensorPorts(trikControl::SensorInterface::Type::digitalSensor).toStdList());
  std::transform(dsensors.begin(), dsensors.end(), std::back_inserter(vsh), initSensors);

  vsh.push_back(std::make_shared<VectorSensorHandle>(brick->accelerometer(), "accelerometer", n));
  vsh.push_back(std::make_shared<VectorSensorHandle>(brick->gyroscope(), "gyroscope", n));
  
  /*  
  
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
    for (const std::shared_ptr<Publisher>& sh : vsh) {
      if (sh) {
	sh->publish();
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
