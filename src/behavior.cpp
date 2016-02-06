#include <ros/ros.h>

#include "bebop_hri/util.h"
#include "bebop_hri/behavior_tools.h"
#include "bebop_hri/behavior.h"

namespace bebop_hri
{

BebopBehaviorNode::BebopBehaviorNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : nh_(nh),
    priv_nh_(priv_nh),
    joy_sub_(nh, "joy", 10),
    status_publisher_(nh, "status"),
    bebop_mode_(constants::MODE_IDLE),
    bebop_prev_mode_(constants::MODE_IDLE),
    bebop_resume_mode_(constants::MODE_IDLE),
    last_transition_time_(ros::Time::now())
{
  UpdateParams();
  bebop_mode_ = static_cast<constants::bebop_mode_t>(param_init_mode_);
}

void BebopBehaviorNode::UpdateParams()
{
  util::get_param(priv_nh_, "update_freq", param_update_rate_, 10.0);
  util::get_param(priv_nh_, "initial_mode", param_init_mode_, 0);
  util::get_param(priv_nh_, "joy_override_buttion", param_joy_override_button_, 7);
  util::get_param(priv_nh_, "joy_override_timeout", param_joy_override_timeout_, 20.0);
  util::get_param(priv_nh_, "idle_timeout", param_idle_timeout_, 10.0);
}

void BebopBehaviorNode::Reset()
{
  ROS_WARN("[BEH] Behavior Reset");
}

void BebopBehaviorNode::UpdateBehavior()
{
  const bool is_transition = (bebop_mode_ != bebop_prev_mode_);
  if (is_transition)
  {
    ROS_WARN("[BEH] State Transitioned (%s -> %s)",
             BEBOP_MODE_STR(bebop_prev_mode_).c_str(),
             BEBOP_MODE_STR(bebop_mode_).c_str());
    last_transition_time_ = ros::Time::now();
  }

  const ros::Duration mode_duration = ros::Time::now() - last_transition_time_;

  status_publisher_ << "Current State: '" << constants::STR_BEBOP_MODE_MAP[bebop_mode_].c_str() << "'";
  status_publisher_ << " Duration: " << mode_duration.toSec();

  bebop_prev_mode_ = bebop_mode_;

  // Deactivate all async subs if they are not fresh enough
  joy_sub_.DeactivateIfOlderThan(1.0);

  // Emergency behavior is implemented way down the pipeline, in cmd_vel_mux layer
  // regardless of the current mode, is joy_override_button is pressed, we will pause execution
  if ((bebop_mode_ != constants::MODE_MANUAL) &&
      joy_sub_.IsActive() && joy_sub_()->buttons.at(param_joy_override_button_))
  {
    ROS_WARN("[BEH] Joystick override detected");
    bebop_resume_mode_ = bebop_mode_;
    bebop_mode_ = constants::MODE_MANUAL;
  }

  switch (bebop_mode_)
  {
  case constants::MODE_IDLE:
  {
    if (is_transition)
    {
      Reset();
    }
    if (mode_duration.toSec() > param_idle_timeout_)
    {
      bebop_mode_ = static_cast<constants::bebop_mode_t>(param_init_mode_);

      // Avoid deadlock, if the user has not specified a default starting state,
      // perfrom searching
      if (bebop_mode_ == constants::MODE_IDLE)
      {
        bebop_mode_ = constants::MODE_SEARCHING;
      }

      ROS_INFO_STREAM("[BEH] Idle timeout, transitioning to initial state: " <<
                      BEBOP_MODE_STR(bebop_mode_));
    }
    break;
  }

  case constants::MODE_MANUAL:
  {
    if (mode_duration.toSec() > param_joy_override_timeout_ &&
        bebop_resume_mode_ != constants::MODE_IDLE)
    {
      ROS_WARN_STREAM("[BEH] Time in manual mode exceeded the `joy_override_timeout` of " << param_joy_override_timeout_);
      ROS_WARN_STREAM("[BEH] Behavior will be reset after override is over");
      bebop_resume_mode_ = constants::MODE_IDLE;
    }
    if (joy_sub_.IsActive() && !joy_sub_()->buttons[param_joy_override_button_])
    {
      ROS_WARN_STREAM("[BEH] Joystick override ended, going back to " << BEBOP_MODE_STR(bebop_resume_mode_));
      bebop_mode_ = bebop_resume_mode_;
    }
    break;
  }
  case constants::MODE_NUM:
  {
    ROS_FATAL("WTF!");
  }
  }

  ROS_DEBUG_THROTTLE(1 , "[BEH] %s", status_publisher_.GetBuffer().str().c_str());
  status_publisher_.Publish();
}

void BebopBehaviorNode::Spin()
{
  ros::Rate loop_rate(param_update_rate_);

  while (ros::ok())
  {
    try
    {
      UpdateBehavior();
      if (!loop_rate.sleep())
      {
        ROS_WARN_STREAM("[BEH] Loop frequency of " << param_update_rate_ << " missed!");
      }
      ros::spinOnce();
    }
    catch (const ros::Exception& e)
    {
      ROS_ERROR_STREAM("[BEH] ROS error: " << e.what());
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("[BEH] Runtime error: " << e.what());
    }
  }

  ROS_INFO_STREAM("[BEH] Exiting the main loop");
}

}  // namespace bebop_hri

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_behavior_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  double update_freq;
  int32_t initial_state;

  ROS_INFO_STREAM("[BEH] Starting behavior node ...");
  bebop_hri::BebopBehaviorNode bebop_behavior(nh, priv_nh);
  bebop_behavior.Spin();
}
