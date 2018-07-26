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
//   ROS_DEBUG_STREAM(" init X0 is \n" << "r:" << efk_.X_.r.transpose() << ',' <<
//         "\nq:" << efk_.X_.q.coeffs().transpose() << ',' <<
//         "\nv:" << efk_.X_.v.transpose() << ',' <<
//         "\nw:" << efk_.X_.w.angle() << '-' << efk_.X_.w.axis().transpose());
//   ROS_DEBUG_STREAM(" init P is \n" << efk_.P_);

  // setup subscribers and publishers
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &Tracker::cameraInfoCallback, this);
  starting_pose_sub_ = nh_.subscribe("camera_pose", 1, &Tracker::cameraPoseCallback, this);
  reset_sub_ = nh_.subscribe("reset", 1, &Tracker::resetCallback, this);
  event_sub_ = nh_.subscribe("events", 10, &Tracker::eventsCallback, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tracked_pose", 2, true);
  image_transport::ImageTransport it_(nh_);
  event_map_pub_ = it_.advertise("map_events", 1);
}

Tracker::~Tracker() {
    pose_pub_.shutdown();
    event_map_pub_.shutdown();
}

void Tracker::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    ROS_INFO("got camera info");
    got_camera_info_ = true;
    
    // K is row-major matrix
    camera_matrix_ << msg->K[2], msg->K[5], msg->K[0], msg->K[4];

    camera_matrix_cv = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            camera_matrix_cv.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

    dist_coeffs_cv = cv::Mat(msg->D.size(), 1, CV_64F);
    for (int i = 0; i < msg->D.size(); i++)
        dist_coeffs_cv.at<double>(i) = msg->D[i];

    ROS_DEBUG_STREAM("camera matrix: \n" << camera_matrix_cv << "\n dist coeffs: \n" << dist_coeffs_cv);

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
    is_tracking_running_ = false;

    // create initial state from last camera pose
    EFK::State X0;
    X0.r = camera_position_;
    X0.q = camera_orientation_;
    X0.v = Vec3::Zero();
    X0.w = AngleAxis(0, Vec3::UnitZ());
    efk_.init(X0);

    // reset time
    last_event_ts = ros::Time(0);

    // reset counter
    event_counter_ = 0;

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
        const dvs_msgs::Event &e = msg->events[i];
        Tracker::Event event { Point2d(e.x, e.y), e.ts };
        // undistort event
        undistortEvent(event);
        handleEvent(event);
    }
}

void Tracker::publishTrackedPose(const EFK::State& S) {
    ROS_DEBUG("publishing tracker pose");

    geometry_msgs::PoseStamped poseStamped;

    poseStamped.header.frame_id="map";
    poseStamped.header.stamp = ros::Time::now();

    poseStamped.pose.position.x = S.r[0];
    poseStamped.pose.position.y = S.r[1];
    poseStamped.pose.position.z = S.r[2];
    
    poseStamped.pose.orientation.x = S.q.x();
    poseStamped.pose.orientation.y = S.q.y();
    poseStamped.pose.orientation.z = S.q.z();
    poseStamped.pose.orientation.w = S.q.w();

    pose_pub_.publish(poseStamped);
}


void Tracker::updateMapEvents(const Tracker::Event &e, bool used) {
    // publish map with event
    // get projected map
    if (event_counter_ == 0)
        map_events_ = cv::Mat(180, 240, CV_8UC3, cv::Scalar(0,0,0));
    // add event red if used, grey if not
    // event is a float !!!
    cv::Point event_point(round(e.p[0]), round(e.p[1]));
    if (event_point.x >= 0 and event_point.y >= 0 and 
        event_point.x < map_events_.cols and event_point.y < map_events_.rows) {
        map_events_.at<cv::Vec3b>(cv::Point(e.p[0], e.p[1])) = (
            used ? cv::Vec3b(0, 0, 255) : cv::Vec3b(150, 150, 150));
    } /*else {
        ROS_DEBUG_STREAM("event point outside image " << e.p.transpose());
    }*/
        
    event_counter_++;

    if (event_counter_ == PUBLISH_MAP_EVENTS_RATE) {
        //map_.draw2dMap(map_events_);
        map_.draw2dMapWithCov(map_events_, efk_.getCovariance().block<7,7>(0,0));
        // convert and publish tracked map
        cv_bridge::CvImage cv_image;
        map_events_.copyTo(cv_image.image);
        cv_image.encoding = "bgr8";
        event_map_pub_.publish(cv_image.toImageMsg());
        // display with pause
        //     cv::imshow("map events", map_events_);
        //     cv::waitKey(0);

        event_counter_ = 0;
    }
}

void displayState(EFK::State S) {
    ROS_DEBUG_STREAM(" state:\n" <<
        "\tr: " << S.r.transpose() << '\n' <<
        "\tq: " << S.q.coeffs().transpose() << '\n' <<
        "\tv: " << S.v.transpose() << '\n' <<
        "\tw: " << S.w.angle() << " :: " << S.w.axis().transpose()
    );
}


void Tracker::handleEvent(const Tracker::Event &e) {
    if (last_event_ts.isZero()) { // first event
        last_event_ts = e.ts;
        return;
    }
    // predict
    double dt = (e.ts - last_event_ts).toSec();
    if (dt > 1e-2) ROS_WARN_STREAM("huge dt " << dt);
    else if (dt > 1e-4) ROS_DEBUG_STREAM("big dt " << dt);
    
    last_event_ts = e.ts;
    ROS_DEBUG("##############################");
    ROS_DEBUG_STREAM("### EVENT " << e.p << " dt = " << dt);
    ROS_DEBUG_STREAM("P diagonal" << efk_.getCovariance().diagonal().transpose());
    ROS_DEBUG("# before prediction");
    displayState(efk_.getState());
    efk_.predict(dt);
    ROS_DEBUG("# after prediction");
    displayState(efk_.getState());

    // associate event to a segment in projected map
    double dist;
    const int segmentId = map_.getNearest(e.p, dist, MATCHING_DIST_THRESHOLD, MATCHING_DIST_MIN_MARGIN);
    
    ROS_DEBUG_STREAM("event is at distance " << dist << ", segment " << segmentId);

    // no segment matched
    if (segmentId < 0) {
        updateMapEvents(e);
        return; // skip event
    }
    updateMapEvents(e, true);

    // reproject associated segment
    EFK::State S = efk_.getState();
    // map_.project(segmentId, S.r, S.q, camera_matrix_);
    // DEBUG PROJECTING ALL
    map_.projectAll(S.r, S.q, camera_matrix_);

    // compute measurement (distance) and jacobian
    Eigen::RowVector3d jac_d_r;
    Eigen::RowVector4d jac_d_q;
    dist = map_.getDistance(e.p, segmentId, jac_d_r, jac_d_q);
    Eigen::Matrix<double, 1, 7> H;
    H << jac_d_r, jac_d_q;
    
    // update state in efk
    efk_.update(dist, H);
    ROS_DEBUG("# after update");
    displayState(efk_.getState());
    publishTrackedPose(efk_.getState());
}


void Tracker::undistortEvent(Tracker::Event &e) {
    // this is probably ineficient cv::undistortPoints is an iterative algorithm
    // should estimate a undistort model once and use it
    ROS_DEBUG_STREAM("before undistort: " << e.p.transpose());
    vector<cv::Point2d> points {cv::Point2d(e.p[0], e.p[1])};

    cv::undistortPoints(points, points,
                        camera_matrix_cv, dist_coeffs_cv,
                        cv::noArray(), camera_matrix_cv);

    e.p << points[0].x, points[0].y;
    ROS_DEBUG_STREAM("after undistort: " << e.p.transpose());
}

} // namespace
