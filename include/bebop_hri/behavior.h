#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <autonomy_leds_msgs/Feedback.h>

// This is a placeholder for obzerver
#include <sensor_msgs/RegionOfInterest.h>
#include "cftld_ros/Track.h"
#include "bebop_vservo/Target.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"

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
  MODE_APPROACHING_LOST = 5,
  MODE_FOLLOWING_PERSON = 6,
  MODE_CLOSERANGE_ENGAGED = 7,
  MODE_MANUAL = 8,
  MODE_BAD_VIDEO = 9,
  MODE_NUM
};

const std::string STR_BEBOP_MODE_MAP[MODE_NUM + 1] =
{
  "Idle", "Searching", "Long-range Engaging", "Long-range Engaged",
  "Approaching The person", "Lost The Person", "Following The Person",
  "Close-range Engaged",
  "Manual (Joy Override)", "Stale Video", "NAN"
};

#define BEBOP_MODE_STR(x) (::bebop_hri::constants::STR_BEBOP_MODE_MAP[x])

namespace color
{
  std_msgs::ColorRGBA green, red, blue, cyan, magenta, yellow, white;
} // constants::namespace color

}  // namespace constants

class BebopBehaviorNode
{
protected:
  // ros stuff
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  behavior_tools::ASyncSub<sensor_msgs::Joy> sub_joy_;
  behavior_tools::ASyncSub<sensor_msgs::RegionOfInterest> sub_periodic_tracks_;
  behavior_tools::ASyncSub<cftld_ros::Track> sub_visual_tracker_track_;
  behavior_tools::ASyncSub<bebop_msgs::Ardrone3PilotingStateAttitudeChanged> sub_bebop_att_;
  behavior_tools::ASyncSub<sensor_msgs::CameraInfo> sub_camera_info_;

  // To reset/initialize the visualal tracker
  ros::Publisher pub_cftld_tracker_reset_;
  ros::Publisher pub_cftld_tracker_init_;

  // To enable/disable visual servo
  ros::Publisher pub_visual_servo_enable_;
  ros::Publisher pub_visual_servo_target_;

  // To enable/disable obzerver
  ros::Publisher pub_obzerver_enable_;

  // To send LED feedback
  ros::Publisher pub_led_feedback_;

  util::StringPublisher status_publisher_;

  // Messages
  std_msgs::Empty msg_empty_;
  std_msgs::Bool msg_bool_;
  bebop_vservo::Target msg_vservo_target_;
  autonomy_leds_msgs::Feedback msg_led_feedback_;

  // internal stuff
  constants::bebop_mode_t bebop_mode_;  // current bebop state
  constants::bebop_mode_t bebop_mode_prev_;  // prev bebop state
  constants::bebop_mode_t bebop_mode_prev_update_; // bebop state in previous timestep

  constants::bebop_mode_t bebop_resume_mode_;

  ros::Time last_transition_time_;

  int view_angle_;

  // params
  double param_update_rate_;
  int32_t param_init_mode_;
  int32_t param_joy_override_button_;
  double param_idle_timeout_;
  double param_joy_override_timeout_;
  double param_target_height_;
  double param_target_dist_ground_;
  double param_servo_desired_depth_;
  double param_stale_video_timeout_;

  void ToggleVisualServo(const bool enable);
  void InitColors();
  void SendFeedback( const constants::bebop_mode_t state, const double value = 0);

  void Reset();
  void UpdateParams();
  void UpdateBehavior();
  inline void Transition(const constants::bebop_mode_t& new_mode)
  {
    bebop_mode_prev_ = bebop_mode_;
    bebop_mode_ = new_mode;
  }

public:
  BebopBehaviorNode(ros::NodeHandle& nh, ros::NodeHandle &priv_nh);
  void Spin();

};

}  // namespace bebop_hri
