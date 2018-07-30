#pragma once
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "efk.h"
#include "tracker_map.h"

using Point2d = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;

using std::vector;

namespace track {

class Tracker {
public:
    struct Event {
        Point2d p;
        ros::Time ts;
    };
    Tracker(ros::NodeHandle & nh);
    virtual ~Tracker();
    
    // uncertainty in movement per second
    const Vec3 sigma_v = (Eigen::Vector3d() << 2, 2, 2).finished();
    const Vec3 sigma_w = (Eigen::Vector3d() << 0.5, 0.5, 0.5).finished();
    // uncertainty in measurement of pixel-segment distance
    const double sigma_d    = 1;
    // maximum distance to match event to line
    const double MATCHING_DIST_THRESHOLD = 5;
    // minimum margin between 1st and 2nd distance
    const double MATCHING_DIST_MIN_MARGIN = 10;

    const uint IMAGE_WIDTH = 240;
    const uint IMAGE_HEIGHT = 180;
private:
    ros::NodeHandle nh_;
    EFK efk_;
    TrackerMap map_;

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void resetCallback(const std_msgs::Bool::ConstPtr& msg);
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

    void publishTrackedPose(const EFK::State& S);
    
    // DEPENDENCIES
    // pose msg as initial pose
    ros::Subscriber starting_pose_sub_;
    // reset EFK tracking flag
    ros::Subscriber reset_sub_;
    // events from camera
    ros::Subscriber event_sub_;

    // CAMERA INFO
    bool got_camera_info_;
    Vec4 camera_matrix_; // [u0 u1 fx fy]

    /*     opencv camera matrix and coefs
       [fx  0  u0]
       [0  fy  u1]      [k1 k2 p1 p2 k3...]
       [0   0   1]
    */
    cv::Mat camera_matrix_cv, dist_coeffs_cv;
    ros::Subscriber camera_info_sub_;
    // last camera pose
    bool got_camera_pose_;
    Vec3 camera_position_; // x,y,z
    Quaternion camera_orientation_; // quaternion x,y,z,w

    // TRACKING VARIABLES
    // is running ?
    bool is_tracking_running_;
    ros::Time last_event_ts;    
    void handleEvent(const Tracker::Event &e);

    // UNDISTORT EVENTS
    Vec3 undist_coeffs;
    void undistortEvent(Tracker::Event &e);

    // VISUALIZATION
    // publish pose
    ros::Publisher pose_pub_;
    // debug event association
    image_transport::Publisher event_map_pub_;
    // image of map and events
    cv::Mat map_events_;
    // number of events to acumulate before publishing a map image
    const uint PUBLISH_MAP_EVENTS_RATE = 1000;
    uint event_counter_;
    void updateMapEvents(const Tracker::Event &e, bool used = false);
};

}
