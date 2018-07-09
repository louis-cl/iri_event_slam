#pragma once
#include <vector>
#include <Eigen/Dense>

using std::vector;
using Point2d = Eigen::Vector2d;
using Point3d = Eigen::Vector3d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Quaternion = Eigen::Quaterniond;

namespace track
{
class TrackerMap {
    public:
        struct Segment2D {
            Point2d p1, p2;
        };
        struct Segment3D {
            Point3d p1, p2;
        };

        TrackerMap();
        
        // project all 3d segment to the 2d map
        void projectAll(const Vec3& camera_position,
                        const Quaternion& camera_orientation,
                        const Vec4& camera_matrix);
        // project a 3d segment to the 2d map
        void project(uint s_id,
            const Vec3& camera_position,
            const Quaternion& camera_orientation,
            const Vec4& camera_matrix);

        // get distance of a point to a segment in the 2d map
        double getDistance(const Point2d &p, uint s_id);
        // get id of the nearest segment to a 2d point in the 2d map
        uint getNearest(const Point2d &p);

    private:
        vector<Segment3D> world_map_;
        vector<Segment2D> camera_map_;
};

}
