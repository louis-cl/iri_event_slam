#include "tracker/tracker.h"

namespace track
{

Tracker::Tracker(ros::NodeHandle & nh) : nh_(nh) {
  got_camera_info_ = false;
  got_camera_pose_ = false;
  is_tracking_running_ = false;

  // setup subscribers and publishers
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &Tracker::cameraInfoCallback, this);
  starting_pose_sub_ = nh_.subscribe("camera_pose", 1, &Tracker::cameraPoseCallback, this);
  reset_sub_ = nh_.subscribe("reset", 1, &Tracker::resetCallback, this);
  event_sub_ = nh_.subscribe("events", 1, &Tracker::eventsCallback, this);
}

Tracker::~Tracker() {}

void Tracker::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
  ROS_INFO("got camera info");
  got_camera_info_ = true;

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    dist_coeffs_.at<double>(i) = msg->D[i];

  camera_info_sub_.shutdown();
}

void Tracker::cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    got_camera_pose_ = true;
    camera_position_ = cv::Vec3d(msg->pose.position.x,
                                 msg->pose.position.y,
                                 msg->pose.position.z);
    camera_orientation_ = cv::Vec4d(msg->pose.orientation.x,
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z,
                                    msg->pose.orientation.w);
    ROS_DEBUG_STREAM("got pose " << camera_position_ << " and orientation " << camera_orientation_);
}

void Tracker::resetCallback(const std_msgs::Bool::ConstPtr& msg) {
    ROS_DEBUG("received reset callback!");
}

void Tracker::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
    ROS_DEBUG("got an event array of size %lu", msg->events.size());
    if (!is_tracking_running_) return;
}

} // namespace
