#include <ros/ros.h>
#include "track_init/track_init.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "track_init");

  ros::NodeHandle nh;

  track::TrackInit tracker_init(nh);
  ROS_INFO("started track_init");
  ros::spin();

  return 0;
}