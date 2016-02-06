#include <std_msgs/String.h>
#include "bebop_hri/util.h"

namespace bebop_hri
{
namespace util
{

StringPublisher::StringPublisher(const ros::NodeHandle& _nh, const std::string& _topic)
  : active(true), nh(_nh), topic(_topic), buffer("")
{
  publisher = nh.advertise<std_msgs::String>(topic, 25);
}

StringPublisher::StringPublisher(): active(false), buffer("")
{}

void StringPublisher::Init(const ros::NodeHandle& _nh, const std::string& _topic)
{
  nh = _nh;
  topic = _topic;
  active = true;
}

StringPublisher::~StringPublisher()
{
  buffer.str("");
}

const std::stringstream& StringPublisher::GetBuffer() const
{
  return buffer;
}
void StringPublisher::ResetBuffer()
{
  buffer.str("");
}

void StringPublisher::Append(const std::string& str)
{
  buffer << str;
}

void StringPublisher::Append(const double d)
{
  buffer << std::setprecision(3) << d;
}

void StringPublisher::Append(const int i)
{
  buffer << i;
}

void StringPublisher::Append(const std::string& str, const double d)
{
  Append(str);
  Append(d);
}

void StringPublisher::Append(const std::string &str, const int i)
{
  Append(str);
  Append(i);
}

StringPublisher& StringPublisher::operator <<(const StringPublisher& rhs)
{
  Append(rhs.GetBuffer().str());
  return *this;
}

StringPublisher& StringPublisher::operator<<(const std::string &str)
{
  Append(str);
  return *this;
}

StringPublisher& StringPublisher::operator<<(const double d)
{
  Append(d);
  return *this;
}

StringPublisher& StringPublisher::operator<<(const int i)
{
  Append(i);
  return *this;
}

void StringPublisher::Publish(const bool checkSubscribers)
{
  if (!active)
  {
    ROS_ERROR("Publisher has not been initialized!");
    return;
  }

  if ((!checkSubscribers) || (publisher.getNumSubscribers() > 0))
  {
    std_msgs::String data;
    data.data = buffer.str();
    publisher.publish(data);
  }
  ResetBuffer();
}

void StringPublisher::Publish(const std::string str)
{
  buffer.str(str);
  Publish();
}

}  // namespace util
}  // namespace bebop_hri
