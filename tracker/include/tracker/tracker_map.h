#pragma once
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include "slam_line.h"

using std::vector;
using std::abs;
using Point2d = Eigen::Vector2d;
using Point3d = Eigen::Vector3d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Quaternion = Eigen::Quaterniond;

namespace track
{
class TrackerMap {
    public:
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
        // same as above + jacobians with respect to r,q
        double getDistance(const Point2d& p, uint s_id,
                Eigen::RowVector3d& jac_d_r, Eigen::RowVector4d& jac_d_q);

        // get id of the nearest segment to a 2d point in the 2d map
        uint getNearest(const Point2d &p);
        uint getNearest(const Point2d &p, double &best_distance);

    private:
        vector<SlamLine> map_;
};

}
