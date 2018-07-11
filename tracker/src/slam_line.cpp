#include "tracker/slam_line.h"
#include <iostream>

namespace track
{

SlamLine::SlamLine(const Point3d& p1, const Point3d& p2) :
    p1_3d(p1), p2_3d(p2) {}

void SlamLine::project(const Vec3& r, const Quaternion& q, const Vec4& K) {
    Eigen::Matrix3d R = q.toRotationMatrix().transpose();
    // 3d world segments -> 3d camera segments
    Point3d p1_3d_c = R * (p1_3d - r);
    Point3d p2_3d_c = R * (p2_3d - r);
    // 3d camera segments -> 2d camera segments
    double fx = K[0];
    double fy = K[1];
    double u0 = K[2];
    double u1 = K[3];
    
    p1_2d[0] = fx * p1_3d_c[0]/p1_3d_c[2] + u0;
    p1_2d[1] = fy * p1_3d_c[1]/p1_3d_c[2] + u1;
    p2_2d[0] = fx * p2_3d_c[0]/p2_3d_c[2] + u0;
    p2_2d[1] = fy * p2_3d_c[1]/p2_3d_c[2] + u1;
    // 2d camera segments -> 2d line hm
    // (x,y) & (a,b) -> line is (y - b, a - x, bx - ay) "cross product"
    line_2d << p1_2d[1] - p2_2d[1],
               p2_2d[0] - p1_2d[0],
               p2_2d[1]*p1_2d[0] - p2_2d[0]*p1_2d[1];
}

double SlamLine::getDistance(const SlamLine& s, const Point2d& p) {

}

}
