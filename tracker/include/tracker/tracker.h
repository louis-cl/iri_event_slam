#pragma once
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <Eigen/Dense>

#include "efk.h"
#include "tracker_map.h"

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;

namespace track {

class Tracker {
public:
    Tracker(ros::NodeHandle & nh);
    virtual ~Tracker();
    
    // uncertainty in movement per second
    const Vec3 sigma_v = (Eigen::Vector3d() << 2, 2, 2).finished();
    const Vec3 sigma_w = (Eigen::Vector3d() << 2, 2, 2).finished();
    //const Vec3 sigma_v((Eigen::Matrix3d() << 2,2,2).finished());
    //const Vec3 sigma_w((Vec3() << 2,2,2).finished());
    // uncertainty in measurement of pixel-segment distance
    const double sigma_d    = 0.5;

private:
    ros::NodeHandle nh_;
    EFK efk_;
    TrackerMap map_;

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void resetCallback(const std_msgs::Bool::ConstPtr& msg);
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    
    // camera info parameters
    bool got_camera_info_;
    Vec4 camera_matrix_;
    //Mat camera_matrix_, dist_coeffs_;
    ros::Subscriber camera_info_sub_;

    // last camera pose
    bool got_camera_pose_;
    Vec3 camera_position_; // x,y,z
    Quaternion camera_orientation_; // quaternion x,y,z,w

    // is running ?
    bool is_tracking_running_;

    // pose msg as initial pose
    ros::Subscriber starting_pose_sub_;
    // reset EFK tracking
    ros::Subscriber reset_sub_;
    // get events
    ros::Subscriber event_sub_;

    // TRACKING VARIABLES
    ros::Time last_event_ts;
    void handleEvent(const dvs_msgs::Event &e);
};

}
