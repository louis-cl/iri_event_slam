#pragma once
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <vector>
#include <algorithm>

namespace track {

class TrackInit {
public:
  TrackInit(ros::NodeHandle & nh);
  virtual ~TrackInit();

private:
  ros::NodeHandle nh_;

  // get camera info
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  // handle raw grayscale intensity image
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
  // find square in image an return points
  static std::vector<cv::Point> findSquare(const cv_bridge::CvImageConstPtr img);
  // sort points CW
  static void sortPointsCW(std::vector<cv::Point> &points);
  
  bool got_camera_info_;
  cv::Mat camera_matrix_, dist_coeffs_;
  ros::Subscriber camera_info_sub_;

  image_transport::Publisher image_pub_;

  image_transport::Subscriber image_sub_;
};

} // namespace

// build with
// $ catking build track
// run with
// $ rosrun track track_init ige:=/dvs/image_raw