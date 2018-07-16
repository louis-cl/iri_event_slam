#include "tracker/tracker_map.h"

namespace track
{
TrackerMap::TrackerMap() {
    // setup known map, UGLY CODE, do this outside...
    const double hw = 85.0f/2;
    vector<Point3d> model_points {
        Point3d(-hw, -hw, 0.0f),
        Point3d( hw, -hw, 0.0f),
        Point3d( hw,  hw, 0.0f),
        Point3d( -hw,  hw, 0.0f)
    };
    for(int i = 0; i < 4; ++i) {
        Point3d p1 = model_points[i];
        Point3d p2 = model_points[(i+1) % 4];
        map_.push_back(SlamLine(p1,p2));
    }
}
void TrackerMap::projectAll(const Vec3& camera_position,
                            const Quaternion& camera_orientation,
                            const Vec4& camera_matrix) {
    for (SlamLine& sl : map_)
        sl.project(camera_position, camera_orientation, camera_matrix);
}

void TrackerMap::project(uint s_id,
    const Vec3& camera_position,
    const Quaternion& camera_orientation,
    const Vec4& camera_matrix) {
    map_[s_id].project(camera_position, camera_orientation, camera_matrix);
}

double TrackerMap::getDistance(const Point2d &p, uint s_id) {
    return SlamLine::getDistance(map_[s_id], p);
}

double TrackerMap::getDistance(const Point2d& p, uint s_id,
                Eigen::RowVector3d& jac_d_r, Eigen::RowVector4d& jac_d_q) {
    return SlamLine::getDistance(map_[s_id], p, jac_d_r, jac_d_q);
}

uint TrackerMap::getNearest(const Point2d &p, double &best_distance) {
    uint best_id = 0;
    best_distance = abs(SlamLine::getDistance(map_[0], p));
    ROS_INFO_STREAM("observed distance " << best_distance << " to segment 0");
    for (int i = 1; i < map_.size(); ++i) {
        double distance_i = abs(SlamLine::getDistance(map_[i], p));
        ROS_INFO_STREAM("observed distance " << distance_i << " to segment " << i);
        if (distance_i < best_distance) {
            best_id = i;
            best_distance = distance_i;
        }
    }
    return best_id;
}

uint TrackerMap::getNearest(const Point2d &p) {
    double d;
    return getNearest(p, d);
}

cv::Mat TrackerMap::get2dMap(uint h, uint w) {
    cv::Mat img(h, w, CV_8UC3, cv::Scalar(0,0,0));
    for (int i = 0; i < map_.size(); ++i) {
        const SlamLine &sl = map_[i];
        cv::Point p1(sl.p1_2d[0], sl.p1_2d[1]);
        cv::Point p2(sl.p2_2d[0], sl.p2_2d[1]);
        cv::line(img, p1, p2, CV_RGB(0,0,255), 2);
        cv::Point p = (p1+p2) * 0.5;
        cv::putText(img, std::to_string(i), p, cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,125,255), 1);
    }
    return img;
}

}

  