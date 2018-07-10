#pragma once
#include <Eigen/Dense>

using Point2d = Eigen::Vector2d;
using Point3d = Eigen::Vector3d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Quaternion = Eigen::Quaterniond;

namespace track
{

class SlamLine {
    public:
        SlamLine(const Point3d& p1, const Point3d& p2);
        
        // project 3d points to 2d points
        void project(const Vec3& r, const Quaternion& q, const Vec4& K);

        // get distance between a SlamLine and a 2d point
        static double getDistance(const SlamLine& s, const Point2d& p);
    
        // world coordinates points
        Point3d p1_3d;
        Point3d p2_3d;

        // projected points
        Point2d p1_2d;
        Point2d p2_2d;
        
        // homogeneous coordinates
        Point2d line_2d;

        // Jacobian of line_2d wrt to position and orientation in project
        Eigen::Matrix<double, 2, 3> jac_line_2d_r;
        Eigen::Matrix<double, 2, 4> jac_line_2d_q;
};

} // namespace