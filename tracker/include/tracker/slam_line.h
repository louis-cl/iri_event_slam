#pragma once
#include <Eigen/Dense>
#include <cmath>

using std::sqrt;

using Point2d = Eigen::Vector2d;
using Point3d = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;
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
        inline static double getDistance(const SlamLine& s, const Point2d& p) {
            // signed distance between line and point
            // line aX + bY + c = 0,   point x,y
            // d = (ax + by + c)/|(a,b)|   where a,b,c = line_2d homogeneous
            double a = s.line_2d[0];
            double b = s.line_2d[1];
            double c = s.line_2d[2];
            return (a*p[0] + b*p[1] + c)/sqrt(a*a + b*b);
        }
        
        // get distance with jacobians
        static double getDistance(const SlamLine& s, const Point2d& p,
                                   Eigen::RowVector3d& jac_d_r, Eigen::RowVector4d& jac_d_q);
        
        // heuristic to associate point to line
        inline static bool isAligned(const SlamLine& s, const Point2d& p) {
            Vec2 u = s.p2_2d - s.p1_2d;
            Vec2 v = p - s.p1_2d;
            double pos = u.dot(v) / u.squaredNorm();
            return 0 <= pos and pos <= 1;
        }

        // world coordinates points
        Point3d p1_3d;
        Point3d p2_3d;

        // projected points
        Point2d p1_2d;
        Point2d p2_2d;
        
        // homogeneous coordinates of line joining p1_2d, p2_2d
        Point3d line_2d;

        // Jacobian of line_2d wrt to position and orientation in project
        Eigen::Matrix<double, 3, 3> jac_line_2d_r;
        Eigen::Matrix<double, 3, 4> jac_line_2d_q;
};

} // namespace