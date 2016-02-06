#ifndef BEBOP_UTIL_H
#define BEBOP_UTIL_H

#include <ros/ros.h>
#include <string>

namespace bebop_hri
{
namespace util
{

template<class T>
inline void get_param(const ros::NodeHandle& nh,
                      const std::string& param_name, T& var, const T default_value)
{
  nh.param(param_name, var, default_value);
  ROS_INFO_STREAM("[BEH] Param " << param_name << " : " << var);
}

template <typename T>
inline int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

class StringPublisher
{
protected:
  bool active;
  ros::NodeHandle nh;
  std::string topic;
  std::stringstream buffer;
  ros::Publisher publisher;
public:
  StringPublisher(const ros::NodeHandle& _nh, const std::string& _topic);
  StringPublisher();
  void Init(const ros::NodeHandle& _nh, const std::string& _topic);
  ~StringPublisher();

  const std::stringstream& GetBuffer() const;
  void ResetBuffer();

  void Append(const std::string& str);
  void Append(const double d);
  void Append(const int i);
  void Append(const std::string& str, const double d);
  void Append(const std::string &str, const int i);

  StringPublisher& operator<<(const StringPublisher& rhs);
  StringPublisher& operator<<(const std::string &str);
  StringPublisher& operator<<(const double d);
  StringPublisher& operator<<(const int i);

  void Publish(const bool checkSubscribers = true);
  void Publish(const std::string str);
};

}  // namespace util
}  // namespace bebop_hri

#endif  // BEBOP_UTIL_H
