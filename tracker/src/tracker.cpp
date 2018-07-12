#include "tracker/tracker.h"

namespace track
{

Tracker::Tracker(ros::NodeHandle & nh) : nh_(nh) {
  got_camera_info_ = false;
  got_camera_pose_ = false;
  is_tracking_running_ = false;

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

  // K is row-major matrix
  camera_matrix_ << msg->K[0], msg->K[4], msg->K[2], msg->K[5];

// ignoring distortion for now
//   dist_coeffs_ = Mat(msg->D.size(), 1, CV_64F);
//   for (int i = 0; i < msg->D.size(); i++)
//     dist_coeffs_.at<double>(i) = msg->D[i];

  camera_info_sub_.shutdown();
}

void Tracker::cameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!got_camera_pose_) ROS_INFO("got camera pose");
    got_camera_pose_ = true;
    camera_position_ = Vec3(msg->pose.position.x,
                                 msg->pose.position.y,
                                 msg->pose.position.z);
    camera_orientation_ = Quaternion(msg->pose.orientation.w,
                                    msg->pose.orientation.x,
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z);
    ROS_DEBUG_STREAM("got pose " << camera_position_ << " and orientation " << camera_orientation_.coeffs());
}

void Tracker::resetCallback(const std_msgs::Bool::ConstPtr& msg) {
    ROS_INFO("received reset callback!");
    // create initial state from last camera pose
    EFK::State X0;
    X0.r = camera_position_;
    X0.q = camera_orientation_;
    X0.v = Vec3::Zero();
    X0.w = AngleAxis(0, Vec3::UnitZ());
    efk_.init(X0);

    // reset time
    last_event_ts = ros::Time(0);

    // project map
    map_.projectAll(camera_position_, camera_orientation_, camera_matrix_);

    // put flag at then so that efk is initialized
    is_tracking_running_ = true;
}

void Tracker::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
    ROS_DEBUG("got an event array of size %lu", msg->events.size());
    if (!(is_tracking_running_ and got_camera_pose_ and got_camera_info_)) return;
    // handle tracking
    for (int i = 0; i < msg->events.size(); ++i) {
        handleEvent(msg->events[i]);
    }
}

void Tracker::handleEvent(const dvs_msgs::Event &e) {
    if (last_event_ts.isZero()) { // first event
        last_event_ts = e.ts;
        return;
    }
    // predict
    double dt = (e.ts - last_event_ts).toSec();
    ROS_DEBUG_STREAM(" event: dt = " << dt);
    efk_.predict(dt);

    Point2d eventPoint(e.x, e.y);
    // associate event to a segment in projected map
    double dist;
    const uint segmentId = map_.getNearest(eventPoint, dist);
    ROS_INFO_STREAM("observed distance " << dist << " to segment " << segmentId);
    
    // reproject associated segment
    EFK::State S = efk_.getState();
    map_.project(segmentId, S.r, S.q, camera_matrix_);
    
    
    if (abs(dist) >= THRESHOLD_MATCHING_DIST) return;

    // compute measurement (distance) and jacobian
    Eigen::RowVector3d jac_d_r;
    Eigen::RowVector4d jac_d_q;
    dist = map_.getDistance(eventPoint, segmentId, jac_d_r, jac_d_q);
    Eigen::Matrix<double, 1, 7> H;
    H << jac_d_r, jac_d_q;
    
    // update state in efk
    efk_.update(dist, H);
}

} // namespace
