#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "bebop_hri/util.h"
#include "bebop_hri/behavior_tools.h"
#include "bebop_hri/behavior.h"

namespace bebop_hri
{

BebopBehaviorNode::BebopBehaviorNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : nh_(nh),
    priv_nh_(priv_nh),
    sub_joy_(nh_, "joy", 10),
    sub_periodic_tracks_(nh_, "periodic_tracks", 1),
    sub_visual_tracker_track_(nh_, "visual_tracker_track", 1),
    sub_bebop_att_(nh_, "bebop/states/ARDrone3/PilotingState/AttitudeChanged", 10),
    sub_camera_info_(nh_, "bebop/camera_info", 1),
    pub_cftld_tracker_reset_(nh_.advertise<std_msgs::Empty>("visual_tracker_reset", 1, true)),
    pub_cftld_tracker_init_(nh_.advertise<sensor_msgs::RegionOfInterest>("visual_tracker_init", 1, true)),
    pub_visual_servo_enable_(nh_.advertise<std_msgs::Bool>("visual_servo_enable", 1, true)),
    pub_visual_servo_target_(nh_.advertise<bebop_vservo::Target>("visual_servo_target", 1, true)),
    status_publisher_(nh_, "status"),
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
  util::get_param(priv_nh_, "servo_desired_depth", param_servo_desired_depth_, 2.5);
  util::get_param(priv_nh_, "target_height", param_target_height_, 0.5);
  util::get_param(priv_nh_, "target_dist_ground", param_target_dist_ground_, 0.75);
  util::get_param(priv_nh_, "stale_video_timeout", param_stale_video_timeout_, 10.0);
}

void BebopBehaviorNode::Reset()
{
  ROS_WARN("[BEH] Behavior Reset");
  pub_cftld_tracker_reset_.publish(msg_empty_);
  ToggleVisualServo(false);
}

void BebopBehaviorNode::ToggleVisualServo(const bool enable)
{
  std_msgs::Bool bool_msg;
  bool_msg.data = enable;
  pub_visual_servo_enable_.publish(bool_msg);
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

  ROS_DEBUG_THROTTLE(1 , "[BEH] %s", status_publisher_.GetBuffer().str().c_str());
  status_publisher_.Publish();

  // Deactivate all async subs if they are not fresh enough
  sub_joy_.DeactivateIfOlderThan(1.0);
  sub_periodic_tracks_.DeactivateIfOlderThan(1.0);
  sub_visual_tracker_track_.DeactivateIfOlderThan(5.0);
  sub_bebop_att_.DeactivateIfOlderThan(1.0);
  // The tolerance on camera_info_sub is lower
  sub_camera_info_.DeactivateIfOlderThan(0.5);

  // Emergency behavior is implemented way down the pipeline, in cmd_vel_mux layer
  // regardless of the current mode, is joy_override_button is pressed, we will pause execution
  if ((bebop_mode_ != constants::MODE_MANUAL) &&
      sub_joy_.IsActive() && sub_joy_()->buttons.at(param_joy_override_button_))
  {
    ROS_WARN("[BEH] Joystick override detected");
    bebop_resume_mode_ = bebop_mode_;
    bebop_mode_ = constants::MODE_MANUAL;
    return;
  }

  if ((bebop_mode_ != constants::MODE_MANUAL) &&
      (bebop_mode_ != constants::MODE_BAD_VIDEO) &&
      (bebop_mode_ != constants::MODE_IDLE) &&
      (false == sub_camera_info_.IsActive()))
  {
    ROS_ERROR("[BEH] Video feed is stale");
    bebop_resume_mode_ = bebop_mode_;
    bebop_mode_ = constants::MODE_BAD_VIDEO;
    return;
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
    if (sub_joy_.IsActive() && !sub_joy_()->buttons[param_joy_override_button_])
    {
      ROS_WARN_STREAM("[BEH] Joystick override ended, going back to " << BEBOP_MODE_STR(bebop_resume_mode_));
      bebop_mode_ = bebop_resume_mode_;
    }
    break;
  }

  case constants::MODE_BAD_VIDEO:
  {
    if (is_transition)
    {
      ROS_ERROR("[BEH] Video STALE mode");
      ToggleVisualServo(false);
    }

    if (mode_duration.toSec() > param_stale_video_timeout_)
    {
      ROS_WARN_STREAM("[BEH] Time in video stale mode exceeded the `stale_video_timeout` of " << param_stale_video_timeout_);
      ROS_WARN_STREAM("[BEH] Behavior will be reset after video comes back online");
      bebop_resume_mode_ = constants::MODE_IDLE;
      break;
    }

    if (sub_camera_info_.GetFreshness().toSec() < 0.05)
    {
      ROS_WARN_STREAM("[BEH] Video is good again!");
      bebop_mode_ = bebop_resume_mode_;
      break;
    }

    break;
  }
  case constants::MODE_SEARCHING:
  {
    if (is_transition)
    {
      ; // TODO: Enable obzerver
    }
    if (sub_periodic_tracks_.IsActive())
    {
      const sensor_msgs::RegionOfInterest& roi = sub_periodic_tracks_.GetMsgCopy();
      ROS_INFO_STREAM("[BEH] Obzerver found a stationary periodic track [x, y, w, h]: "
                      << roi.x_offset << " " << roi.y_offset << " " << roi.width << " " << roi.height);
      pub_cftld_tracker_init_.publish(roi);
    }

    // TODO: Add confidence
    // TODO: Reset obzerver
    if (sub_visual_tracker_track_.IsActive() &&
        sub_visual_tracker_track_()->status == cftld_ros::Track::STATUS_TRACKING)
    {
      const cftld_ros::Track& t = sub_visual_tracker_track_.GetMsgCopy();
      ROS_INFO_STREAM("[BEH] Visual tracker has been initialized. Approaching her ... id: "
                      <<  t.uid << " confidence: " << t.confidence);
      bebop_mode_ = constants::MODE_APPROACHING_PERSON;
    }
    break;
  }

  case constants::MODE_APPROACHING_PERSON:
  {
    if (is_transition)
    {
      ROS_INFO_STREAM("[BEH] Enabling visual servo ...");
      ToggleVisualServo(true);
    }

    // Visual tracker's inactiviy is either caused by input stream's being stale or
    // a crash. The former needs a seperate recovery case since this node can also detects it.
    if (!sub_visual_tracker_track_.IsActive())
    {
      ROS_ERROR_STREAM("[BEH] Visual tracker is stale, this should never happen.");

      // TODO: Disable visual servo
      bebop_mode_ = constants::MODE_APPROACHING_LOST;
      break;
    }

    const cftld_ros::Track& t = sub_visual_tracker_track_.GetMsgCopy();

    // TODO: Add confidence
    if (t.status != cftld_ros::Track::STATUS_TRACKING)
    {
      ROS_WARN("[BEH] Tracker has lost the person");
      bebop_mode_ = constants::MODE_APPROACHING_LOST;
    }
    else
    {
      msg_vservo_target_.header.stamp = ros::Time::now();
      msg_vservo_target_.reinit = is_transition;
      msg_vservo_target_.desired_depth = param_servo_desired_depth_;
      msg_vservo_target_.target_distance_ground = param_target_dist_ground_;
      msg_vservo_target_.target_height = param_target_height_;
      msg_vservo_target_.roi = t.roi;
      // vservo node will cache this value on its first call or when reinit=true
      // ignores it all other time
      msg_vservo_target_.desired_yaw_rad = -sub_bebop_att_()->yaw;
      pub_visual_servo_target_.publish(msg_vservo_target_);
    }

    break;
  }

  case constants::MODE_APPROACHING_LOST:
  {
    if (is_transition)
    {
      ROS_INFO_STREAM("[BEH] Target lost during approach, waiting for a while ...");
      ToggleVisualServo(false);
    }

    if (mode_duration.toSec() > 10.0)
    {
      ROS_WARN("[BEH] Recovery failed");
      bebop_mode_ = constants::MODE_IDLE;
    }

    if (sub_visual_tracker_track_.IsActive() &&
        sub_visual_tracker_track_()->status == cftld_ros::Track::STATUS_TRACKING)
    {
      ROS_INFO("[BEH] Recovery has been succesful");
      bebop_mode_ = constants::MODE_APPROACHING_PERSON;
    }
    break;
  }

  case constants::MODE_NUM:
  {
    ROS_FATAL("WTF!");
  }
  }

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
