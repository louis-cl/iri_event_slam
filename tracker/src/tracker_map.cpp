#include "tracker/tracker_map.h"

namespace track
{
TrackerMap::TrackerMap() {
    // setup known map, UGLY CODE, do this outside...
    const double hw = 0.085/2;
    vector<Point3d> model_points {
        Point3d(-hw, -hw, 0.0f),
        Point3d(-hw,  hw, 0.0f),
        Point3d( hw, -hw, 0.0f),
        Point3d( hw,  hw, 0.0f)
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
    best_distance = SlamLine::getDistance(map_[0], p);
    for (int i = 1; i < map_.size(); ++i) {
        double distance_i = SlamLine::getDistance(map_[i], p);
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

}

  