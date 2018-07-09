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
        world_map_.push_back(TrackerMap::Segment3D {p1,p2});
    }
}
void TrackerMap::projectAll(const Vec3& camera_position,
                            const Quaternion& camera_orientation,
                            const Vec4& camera_matrix) {


}

void TrackerMap::project(uint s_id,
    const Vec3& camera_position,
    const Quaternion& camera_orientation,
    const Vec4& camera_matrix) {
    
}

double TrackerMap::getDistance(const Point2d &p, uint s_id) {

}

uint TrackerMap::getNearest(const Point2d &p) {

}

}

  