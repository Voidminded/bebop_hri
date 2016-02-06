#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "bebop_hri/util.h"
#include "bebop_hri/behavior_tools.h"

namespace bebop_hri
{

namespace constants
{

enum bebop_mode_t
{
  MODE_IDLE = 0,
  MODE_SEARCHING = 1,
  MODE_LONGRANGE_ENGAGING = 2,
  MODE_LONGRANGE_ENGAGED = 3,
  MODE_APPROACHING_PERSON = 4,
  MODE_FOLLOWING_PERSON = 5,
  MODE_CLOSERANGE_ENGAGED = 6,
  MODE_MANUAL = 7,
  MODE_NUM
};

const std::string STR_BEBOP_MODE_MAP[MODE_NUM + 1] =
{
  "Idle", "Searching", "Long-range Engaging", "Long-range Engaged",
  "Approaching The person", "Following The person", "Close-range Engaged",
  "Manual (Joy Override)","NAN"
};

#define BEBOP_MODE_STR(x) (::bebop_hri::constants::STR_BEBOP_MODE_MAP[x])

}  // namespace constants



class BebopBehaviorNode
{
protected:
  // ros stuff
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  behavior_tools::ASyncSub<sensor_msgs::Joy> joy_sub_;

  util::StringPublisher status_publisher_;

  // internal stuff
  constants::bebop_mode_t bebop_mode_;
  constants::bebop_mode_t bebop_prev_mode_;
  constants::bebop_mode_t bebop_resume_mode_;

  ros::Time last_transition_time_;

  // params
  double param_update_rate_;
  int32_t param_init_mode_;
  int32_t param_joy_override_button_;
  double param_idle_timeout_;
  double param_joy_override_timeout_;

  void Reset();
  void UpdateParams();
  void UpdateBehavior();

public:
  BebopBehaviorNode(ros::NodeHandle& nh, ros::NodeHandle &priv_nh);
  void Spin();

};

}  // namespace bebop_hri
