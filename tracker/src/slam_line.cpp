#include "tracker/slam_line.h"

namespace track
{

SlamLine::SlamLine(const Point3d& p1, const Point3d& p2) :
    p1_3d(p1), p2_3d(p2) {}

void SlamLine::project(const Vec3& r, const Quaternion& q, const Vec4& K) {
    // *** PROJECTION ***
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

    // *** JACOBIANS ***
    // jacobian of line_2d wrt r
    // L_r = L_pw * PW_pc * PC_r  L = line_2d, PW/PC = point in window (2d) / camera (3d)
    /*
               [  0   1   0   -1 ]
        L_pw = [ -1   0   1    0 ]    |    pw = [x y a b] = (x,y)(a,b)
               [  b  -a  -y    x ]

                                                    [ fx/z  0      -fx*x/z^2 ]
        PW_pc = diag(P(pc1), P(pc2))    |  P(x,y,z) = [ 0     fy/z   -fy*y/z^2 ]

        PC_r  = [-R;-R]

    */
    Eigen::Matrix<double, 3, 4> L_pw;
    L_pw <<  0,  1,  0, -1,
            -1,  0,  1,  0,
            p2_2d[1], -p2_2d[0], -p1_2d[1], p1_2d[0];

    // utility function
    auto PW_pc = [&] (const Point3d& p) { return (Eigen::Matrix<double, 2, 3>() <<
        fx/p[2],    0,  -fx*p[0]/(p[2]*p[2]),
        0,    fy/p[2],  -fy*p[1]/(p[2]*p[2])      
     ).finished();};

    jac_line_2d_r = L_pw * (Eigen::Matrix<double, 4, 3>() <<
            -PW_pc(p1_3d_c)*R,
            -PW_pc(p2_3d_c)*R
        ).finished();

    // jacobian of line_2d wrt q
    /*
        Joan Sola, "Towards visual localization, mapping and moving objects
        tracking by a moible robot," PhD dissertation, pages 181-183, Institut
        National Politechnique de Toulouse, 2007.
    */
    Eigen::Matrix<double, 4, 3> PIqc;
    PIqc <<  q.x(),  q.y(),  q.z(),
            q.w(),  q.z(), -q.y(),
           -q.z(),  q.w(),  q.x(),
            q.y(), -q.x(),  q.w();

    // utility
    auto TFq = [&] (const Vec4& sc) { return (Eigen::Matrix<double, 3, 4>() << 
        sc[1],  sc[0], -sc[3],  sc[2],
        sc[2],  sc[3],  sc[0], -sc[1],
        sc[3], -sc[2],  sc[1],  sc[0]
     ).finished();};

    jac_line_2d_q = L_pw * (Eigen::Matrix4d() <<
        PW_pc(p1_3d_c)*TFq(2*PIqc*(p1_3d - r)),
        PW_pc(p2_3d_c)*TFq(2*PIqc*(p2_3d - r))
    ).finished();
}

double SlamLine::getDistance(const SlamLine& s, const Point2d& p) {
    // signed distance between line and point
    // line aX + bY + c = 0,   point x,y
    // d = (ax + by + c)/|(a,b)|   where a,b,c = line_2d homogeneous
    double a = s.line_2d[0];
    double b = s.line_2d[1];
    double c = s.line_2d[2];
    return (a*p[0] + b*p[1] + c)/sqrt(a*a + b*b);
}

double SlamLine::getDistance(const SlamLine& s, const Point2d& p,
                                 Eigen::RowVector3d& jac_d_r, Eigen::RowVector4d& jac_d_q) {
    double a = s.line_2d[0];
    double b = s.line_2d[1];
    double c = s.line_2d[2];

    double x = p[0];
    double y = p[1];

    // JACOBIANS
    // D_r = D_l * L_r
    // D_q = D_l * L_q
    Eigen::RowVector3d D_l;
    double n = sqrt(a*a + b*b);
    double n3 = n*n*n;
    
    D_l << (b*b*x - a*(b*y + c))/n3, (a*a*y - b*(a*x+c))/n3, 1/n;
    jac_d_r = D_l * s.jac_line_2d_r;
    jac_d_q = D_l * s.jac_line_2d_q;
    return (a*p[0] + b*p[1] + c)/n;
}

}
