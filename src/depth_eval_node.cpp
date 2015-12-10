#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <bebop_msgs/Ardrone3CameraStateOrientation.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <angles/angles.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>

#include <boost/thread/lock_guard.hpp>
#include <cmath>

const std::string& vicon_target_topic = "vicon/ar_box/ar_box";
const std::string& vicon_bebop_topic = "vicon/bebop_blue/bebop_blue";

const double height_target_m = 0.5;
const double distground_target_m = 0.75;

bool cinfo_recv;
ros::Time roi_recv_time(0);
double d_truth = -1.0;
double cam_tilt_rad = 0.0;

double fov_x = 0.0;
double fov_y = 0.0;
image_geometry::PinholeCameraModel cam_model;
cv_bridge::CvImagePtr cv_img_ptr;
sensor_msgs::RegionOfInterest roi;

ros::Publisher d1_pub;
ros::Publisher d2_pub;
ros::Publisher dgt_pub;

//class ASyncSub
//{
//private:
//  typedef boost::function<void (const boost::shared_ptr<T const>& data)> callback_t;

//  ros::NodeHandle nh;
//  bool active_;
//  ros::Time last_updated_;
//  std::string topic_;
//  std::size_t queue_size_;
//  callback_t user_callback_;
//  ros::Subscriber sub_;
//  boost::shared_ptr<T const> msg_cptr_;
//  mutable boost::mutex mutex_;

//  void cb(const boost::shared_ptr<T const> &msg_cptr)
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    active_ = true;
//    last_updated_ = ros::Time::now();
//    msg_cptr_ = msg_cptr;
//    if (user_callback_) user_callback_(msg_cptr_);
//  }

//public:
//  ASyncSub(ros::NodeHandle& nh,
//           const std::string& topic,
//           const std::size_t queue_size,
//           callback_t user_callback = 0)
//    : nh(nh), active_(false), last_updated_(0), topic_(topic), user_callback_(user_callback)
//  {
//    sub_ = nh.subscribe<T>(topic_, queue_size, boost::bind(&ASyncSub<T>::cb, this, _1));
//  }

//  T GetMsgCopy() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return *msg_cptr_;
//  }

//  // Not thread safe
//  const boost::shared_ptr<T const>& GetMsgConstPtr() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return msg_cptr_;
//  }

//  // T operator ()() const {return GetMsgCopy();}

//  // Not thread safe?
//  const boost::shared_ptr<T const>& operator()() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return GetMsgConstPtr();
//  }

//  void Deactivate()
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    active_ = false;
//  }

//  void DeactivateIfOlderThan(const double seconds)
//  {
//    if (!IsActive()) return;
//    if (GetFreshness().toSec() > seconds)
//    {
//      ROS_WARN("Information on topic (%s) is older than (%4.2lf) seconds. Deactivating.", topic_.c_str(), seconds);
//      Deactivate();
//    }
//  }

//  bool IsActive() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return active_;
//  }

//  const ros::Time GetLastUpdated() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return last_updated_;
//  }

//  const ros::Duration GetFreshness() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return ros::Time::now() - last_updated_;
//  }
//};

void CameraOrientationCallback(const bebop_msgs::Ardrone3CameraStateOrientationConstPtr& cam_ori_ptr)
{
  cam_tilt_rad = -angles::from_degrees(cam_ori_ptr->tilt);
}

void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
{
  cam_model.fromCameraInfo(cinfo_msg_ptr);
  fov_x = 2.0 * atan2(cam_model.fullResolution().width / 2.0, cam_model.fx());
  fov_y = 2.0 * atan2(cam_model.fullResolution().height/ 2.0, cam_model.fy());

  ROS_INFO_STREAM_ONCE("FOV " << angles::to_degrees(fov_x) << " " << angles::to_degrees(fov_y));
  cinfo_recv = true;
}

void BebopSyncCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr,
                       const bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& att_ptr)

{
  if (!cinfo_recv) return;
  if ((ros::Time::now() - roi_recv_time).toSec() > 1.0)
  {
    ROS_WARN("ROI is too old, skipping");
    return;
  }
  const double im_height_px = static_cast<double>(cam_model.fullResolution().height);
  double pitch_rad = -att_ptr->pitch; // Nose down increase pitch
  double sigma_1 = static_cast<double>(roi.y_offset) / im_height_px * fov_y;
  double sigma_2 = (im_height_px - static_cast<double>(roi.y_offset + roi.height)) / im_height_px * fov_y;

  double alt_m = alt_ptr->altitude;
  double roi_height_px = roi.height;
  double beta_rad = (roi_height_px / im_height_px) * fov_y;

  double d1_m = height_target_m * sin(M_PI_2 - fov_y/2.0 + sigma_2 - cam_tilt_rad) / sin(beta_rad);
  double d2_m = (alt_m  - (height_target_m + distground_target_m)) / sin(fov_y/2.0 + cam_tilt_rad - sigma_2 - beta_rad);

  ROS_INFO_STREAM("Sanity: " << angles::to_degrees(sigma_1 + sigma_2 + beta_rad));
  ROS_INFO_STREAM("Altitude (m): " << alt_m);
  ROS_INFO_STREAM("Pitch (deg): " << angles::to_degrees(pitch_rad));
  ROS_INFO_STREAM("Camera Tilt (deg)" << angles::to_degrees(cam_tilt_rad));
  ROS_INFO_STREAM("Beta (deg): " << angles::to_degrees(beta_rad));
  ROS_INFO_STREAM("ROI height (px): " << roi_height_px);
  ROS_WARN_STREAM("D1  (m): " << d1_m);
  ROS_WARN_STREAM("D2  (m): " << d2_m);
  ROS_WARN_STREAM("DGT (m): " << d_truth);

  std_msgs::Float64 d_msg;
  d_msg.data = d1_m;
  d1_pub.publish(d_msg);
  d_msg.data = d2_m;
  d2_pub.publish(d_msg);
  d_msg.data = d_truth;
  dgt_pub.publish(d_msg);
}

void ViconSyncCallback(const geometry_msgs::TransformStampedConstPtr& target_msg_ptr,
                       const geometry_msgs::TransformStampedConstPtr& bebop_msg_ptr)
{
  tf2::Transform target, bebop;
  tf2::fromMsg(target_msg_ptr->transform, target);
  tf2::fromMsg(bebop_msg_ptr->transform, bebop);

  d_truth = target.getOrigin().distance(bebop.getOrigin());
}


//void CameraCallback(const sensor_msgs::ImageConstPtr& img_msg_ptr,
//                    const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
//{
//  try
//  {
//    cv_img_ptr = cv_bridge::toCvCopy(img_msg_ptr, "bgr8");
//    cv::rectangle(cv_img_ptr->image, cv::Rect(roi.x_offset,
//                                              roi.y_offset,
//                                              roi.width,
//                                              roi.height),
//                  CV_RGB(255, 0, 0), 2);

//    cv::imshow("view", cv_img_ptr->image);
//    cv::waitKey(5);

//  }
//  catch (cv_bridge::Exception& e)
//  {
//    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg_ptr->encoding.c_str());
//  }
//}

void RoiCallback(const sensor_msgs::RegionOfInterestConstPtr& roi_msg_ptr)
{
  roi = *roi_msg_ptr;
  roi_recv_time = ros::Time::now();
}

int main(int argc, char* argv[])
{
  cinfo_recv = false;
  ros::init(argc, argv, "depth_eval_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAltitudeChanged> bebop_alt_sub(nh, "bebop/states/ARDrone3/PilotingState/AltitudeChanged", 1);
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAttitudeChanged> bebop_att_sub(nh, "bebop/states/ARDrone3/PilotingState/AttitudeChanged", 1);

  message_filters::Subscriber<geometry_msgs::TransformStamped> vicon_target_sub(nh, vicon_target_topic, 1);
  message_filters::Subscriber<geometry_msgs::TransformStamped> vicon_bebop_sub(nh, vicon_bebop_topic, 1);

//  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "bebop/image_raw", 1);
//  message_filters::Subscriber<sensor_msgs::RegionOfInterest> roi_sub(nh, "roi", 1);

  typedef message_filters::sync_policies::ApproximateTime<
      bebop_msgs::Ardrone3PilotingStateAltitudeChanged,
      bebop_msgs::Ardrone3PilotingStateAttitudeChanged> BebopSyncPolicy_t;

  typedef message_filters::sync_policies::ApproximateTime<
      geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> ViconSyncPolicy_t;

  message_filters::Synchronizer<BebopSyncPolicy_t>
      bebop_sync(BebopSyncPolicy_t(10), bebop_alt_sub, bebop_att_sub);

  message_filters::Synchronizer<ViconSyncPolicy_t>
      vicon_sync(ViconSyncPolicy_t(10), vicon_target_sub, vicon_bebop_sub);

  bebop_sync.registerCallback(boost::bind(&BebopSyncCallback, _1, _2));
  vicon_sync.registerCallback(boost::bind(&ViconSyncCallback, _1, _2));

//  image_transport::Subscriber camera_sub = it.subscribe("bebop", 1, CameraCallback);
  ros::Subscriber caminfo_sub = nh.subscribe("bebop/camera_info", 10, CameraInfoCallback);

  // ROI is not syncable at the moment since there is no stamp
  ros::Subscriber roi_sub = nh.subscribe("roi", 10, RoiCallback);

  // Camera pitch changes rarely, no need to sync
  ros::Subscriber cam_orientation_sub = nh.subscribe("bebop/states/ARDrone3/CameraState/Orientation", 10, CameraOrientationCallback);


  d1_pub = nh.advertise<std_msgs::Float64>("result/d1", 10);
  d2_pub = nh.advertise<std_msgs::Float64>("result/d2", 10);
  dgt_pub = nh.advertise<std_msgs::Float64>("result/dgt", 10);

//  cv::namedWindow("view");
//  cv::startWindowThread();

  ros::spin();

//  cv::destroyAllWindows();
  return 0;
}
