#include <ros/ros.h>
#include "tracker/tracker.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "tracker");

  ros::NodeHandle nh;

  track::Tracker tracker(nh);
  ROS_INFO("started tracker");
  ros::spin();

  return 0;
}