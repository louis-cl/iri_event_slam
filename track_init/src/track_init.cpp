#include "track_init/track_init.h"

namespace track {
TrackInit::TrackInit(ros::NodeHandle & nh) : nh_(nh) {
  //got_camera_info_ = false;

  // setup subscribers and publishers
  //camera_info_sub_ = nh_.subscribe("camera_info", 1, &TrackInit::cameraInfoCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_sub_ = it_.subscribe("image", 1, &TrackInit::imageCallback, this);
  image_pub_ = it_.advertise("rendering", 1);
  //undistorted_image_pub_ = it_.advertise("dvs_undistorted", 1);
}

TrackInit::~TrackInit() {
  image_pub_.shutdown();
  //undistorted_image_pub_.shutdown();
}
/*
void TrackInit::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_camera_info_ = true;

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    dist_coeffs_.at<double>(i) = msg->D[i];
}
*/

void TrackInit::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert from grayscale to color image
  cv::cvtColor(cv_ptr->image, last_image_, CV_GRAY2BGR);

  // and publish what i receive
  cv_bridge::CvImage cv_image;
  last_image_.copyTo(cv_image.image);
  cv_image.encoding = "bgr8";
  image_pub_.publish(cv_image.toImageMsg());
}

} // namespace