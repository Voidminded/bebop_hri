#ifndef BEBOP_UTIL_H
#define BEBOP_UTIL_H

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/RegionOfInterest.h>

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

float GetROIIntersectArea(const sensor_msgs::RegionOfInterest& r1, const sensor_msgs::RegionOfInterest& r2)
{
  const float r1_x1 = r1.x_offset;
  const float r1_x2 = r1.x_offset + r1.width;
  const float r1_y1 = r1.y_offset;
  const float r1_y2 = r1.y_offset + r1.height;

  const float r2_x1 = r2.x_offset;
  const float r2_x2 = r2.x_offset + r2.width;
  const float r2_y1 = r2.y_offset;
  const float r2_y2 = r2.y_offset + r2.height;

  return std::max(0.0f, std::min(r1_x2, r2_x2) - std::max(r1_x1, r2_x1)) *
      std::max(0.0f, std::min(r1_y2, r2_y2) - std::max(r1_y1, r2_y1));
}

float GetROIOverlap(const sensor_msgs::RegionOfInterest& r1, const sensor_msgs::RegionOfInterest& r2)
{
  const float si = util::GetROIIntersectArea(r1, r2);
  const float s = static_cast<float>(r1.width * r1.height) + static_cast<float>(r2.width * r2.height) - si;

  if (s <= 0.0) return 0.0;

  const float overlap = si / s;
  assert(overlap >= 0.0f & overlap <= 1.0f);

  return overlap;
}

}  // namespace util
}  // namespace bebop_hri

#endif  // BEBOP_UTIL_H
