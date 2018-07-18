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

void TrackerMap::project(int s_id,
    const Vec3& camera_position,
    const Quaternion& camera_orientation,
    const Vec4& camera_matrix) {
    map_[s_id].project(camera_position, camera_orientation, camera_matrix);
}

int TrackerMap::getNearest(const Point2d &p, double &best_distance) {
    int best_id = -1;
    for (int i = 0; i < map_.size(); ++i) {
        const SlamLine &sl = map_[i];
        if (SlamLine::isAligned(sl, p)) {
            double distance_i = SlamLine::getDistance(map_[i], p);
            if (best_id == -1 or abs(distance_i) < abs(best_distance)) {
                best_id = i;
                best_distance = distance_i;
            }
        }
    }
    return best_id;
}

void TrackerMap::draw2dMap(cv::Mat &img) {
    for (int i = 0; i < map_.size(); ++i) {
        const SlamLine &sl = map_[i];
        cv::Point p1(sl.p1_2d[0], sl.p1_2d[1]);
        cv::Point p2(sl.p2_2d[0], sl.p2_2d[1]);
        cv::line(img, p1, p2, CV_RGB(0,0,255), 2);
        cv::Point p = (p1+p2) * 0.5;
        cv::putText(img, std::to_string(i), p, cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,125,255), 1);
    }
}

}

  