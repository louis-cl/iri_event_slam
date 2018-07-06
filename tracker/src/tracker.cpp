#include "tracker/tracker.h"

namespace track
{

Tracker::Tracker(ros::NodeHandle & nh) : nh_(nh) {
  got_camera_info_ = false;
  got_camera_pose_ = false;
  is_tracking_running_ = false;

  // setup efk
  efk_ = EFK(sigma_v, sigma_w, sigma_d);

// **** DEBUG ****
//   EFK::State X0;
//   X0.r << 0,0,0;
//   X0.q = Quaternion(1,0,0,0);
//   X0.v << 1,0,0;
//   X0.w = AngleAxis(0, Vec3::UnitX());
//   efk_.init(X0);
//   ROS_INFO_STREAM(" init X0 is \n" << "r:" << efk_.X_.r.transpose() << ',' <<
//         "\nq:" << efk_.X_.q.coeffs().transpose() << ',' <<
//         "\nv:" << efk_.X_.v.transpose() << ',' <<
//         "\nw:" << efk_.X_.w.angle() << '-' << efk_.X_.w.axis().transpose());
//   ROS_INFO_STREAM(" init P is \n" << efk_.P_);

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

  camera_matrix_ = Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

  dist_coeffs_ = Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    dist_coeffs_.at<double>(i) = msg->D[i];

  camera_info_sub_.shutdown();
}

void Tracker::cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!got_camera_pose_) ROS_INFO("got camera pose");
    got_camera_pose_ = true;
    camera_position_ = Vec3d(msg->pose.position.x,
                                 msg->pose.position.y,
                                 msg->pose.position.z);
    camera_orientation_ = Vec4d(msg->pose.orientation.x,
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z,
                                    msg->pose.orientation.w);
    ROS_DEBUG_STREAM("got pose " << camera_position_ << " and orientation " << camera_orientation_);
}

void Tracker::resetCallback(const std_msgs::Bool::ConstPtr& msg) {
    ROS_DEBUG("received reset callback!");
    // create initial state from last camera pose
    EFK::State X0;
    X0.r << camera_position_[0], camera_position_[1], camera_position_[2];
    X0.q = Quaternion(camera_orientation_[0],
                      camera_orientation_[1],
                      camera_orientation_[2],
                      camera_orientation_[3]);
    X0.v << 0,0,0;
    X0.w = AngleAxis(0, Vec3::UnitZ());
    efk_.init(X0);

    // put flag at then so that efk is initialized
    is_tracking_running_ = true;
}

void Tracker::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
    ROS_DEBUG("got an event array of size %lu", msg->events.size());
    if (!(is_tracking_running_ and got_camera_pose_ and got_camera_info_)) return;
    // handle tracking
    ROS_INFO("I started tracking !!");
    for (int i = 0; i < msg->events.size(); ++i) {
        dvs_msgs::Event e = msg->events[i];
    }
}

} // namespace
