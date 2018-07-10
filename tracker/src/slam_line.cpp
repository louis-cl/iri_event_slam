#include "tracker/slam_line.h"

namespace track
{

SlamLine::SlamLine(const Point3d& p1, const Point3d& p2) :
    p1_3d(p1), p2_3d(p2) {}

void SlamLine::project(const Vec3& r, const Quaternion& q, const Vec4& K) {

}

double SlamLine::getDistance(const SlamLine& s, const Point2d& p) {
    
}

}
