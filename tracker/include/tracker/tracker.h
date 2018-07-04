#pragma once
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <opencv2/opencv.hpp>

namespace track {

class Tracker {
public:
    Tracker(ros::NodeHandle & nh);
    virtual ~Tracker();

private:
    ros::NodeHandle nh_;

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void resetCallback(const std_msgs::Bool::ConstPtr& msg);
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    
    // camera info parameters
    bool got_camera_info_;
    cv::Mat camera_matrix_, dist_coeffs_;
    ros::Subscriber camera_info_sub_;

    // last camera pose
    bool got_camera_pose_;
    cv::Vec3d camera_position_; // x,y,z
    cv::Vec4d camera_orientation_; // quaternion x,y,z,w

    // is running ?
    bool is_tracking_running_;

    // pose msg as initial pose
    ros::Subscriber starting_pose_sub_;
    // reset EFK tracking
    ros::Subscriber reset_sub_;
    // get events
    ros::Subscriber event_sub_;
};

}
