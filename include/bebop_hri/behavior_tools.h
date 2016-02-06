#ifndef BEHAVIOR_TOOLS_H
#define BEHAVIOR_TOOLS_H

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <ros/ros.h>

namespace behavior_tools
{

template<typename T>
class ASyncSub
{
protected:
  typedef boost::function<void (const boost::shared_ptr<T const>& data)> callback_t;

  ros::NodeHandle nh;
  bool active_;
  ros::Time last_updated_;
  std::string topic_;
  std::size_t queue_size_;
  callback_t user_callback_;
  ros::Subscriber sub_;
  boost::shared_ptr<T const> msg_cptr_;
  mutable boost::mutex mutex_;

  void cb(const boost::shared_ptr<T const> &msg_cptr)
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    active_ = true;
    last_updated_ = ros::Time::now();
    msg_cptr_ = msg_cptr;
    if (user_callback_) user_callback_(msg_cptr_);
  }

public:
  ASyncSub(ros::NodeHandle& nh,
           const std::string& topic,
           const std::size_t queue_size,
           callback_t user_callback = 0)
    : nh(nh), active_(false), last_updated_(0), topic_(topic), user_callback_(user_callback)
  {
    sub_ = nh.subscribe<T>(topic_, queue_size, boost::bind(&ASyncSub<T>::cb, this, _1));
  }

  T GetMsgCopy() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return *msg_cptr_;
  }

  // Not thread safe
  const boost::shared_ptr<T const>& GetMsgConstPtr() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return msg_cptr_;
  }

  // T operator ()() const {return GetMsgCopy();}

  const boost::shared_ptr<T const>& operator()() const
  {
    return GetMsgConstPtr();
  }

  void Deactivate()
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    active_ = false;
  }

  void DeactivateIfOlderThan(const double& seconds)
  {
    if (!IsActive()) return;
    if (GetFreshness().toSec() > seconds)
    {
      ROS_WARN("Information on topic (%s) is older than (%4.2lf) seconds. Deactivating.", topic_.c_str(), seconds);
      Deactivate();
    }
  }

  bool IsActive() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return active_;
  }

  const ros::Time GetLastUpdated() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return last_updated_;
  }

  const ros::Duration GetFreshness() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return ros::Time::now() - last_updated_;
  }
};

}  // namespace behavior_tools

#endif  // BEHAVIOR_TOOLS_H
