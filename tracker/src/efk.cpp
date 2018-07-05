#include "tracker/efk.h"

namespace track
{

EFK::EFK() {}

EFK::EFK(const Vec3& sigma_v, const Vec3& sigma_w, double sigma_d) {
    P_ = Mat13::Zero();
    Q_ = Mat13::Zero();
    Q_.diagonal() << 0,0,0 , 0,0,0,0, sigma_v.cwiseAbs2(), sigma_w.cwiseAbs2();
    R_ = sigma_d*sigma_d;
}

void EFK::init(const State& X0) {
    X_ = X0;
}

void EFK::predict(double dt) {
    /* constant velocity model
        r = r + v * dt
        q = q . q(w*dt)
        v = v
        w = w
    */
    
    // update state X_
    X_.r += X_.v * dt;
    
    Quaternion qw;
    qw = AngleAxis(X_.w.angle()*dt, X_.w.axis());
    X_.q *= qw;

    // update covariance P_
    /*
    F_x =   1       0       dt      0 
            0       Fq_q    0       Fw_q
            0       0       1       0
            0       0       0       1
    */

    Mat13 F_x(Mat13::Identity());
    F_x.block<3,3>(0,7).diagonal().fill(dt); // Fr_v
    F_x.block<4,4>(3,3) = quaternionProductMatrix(qw, false);  // Fq_q
    //F_x.block<4,3>(3,10) = Fw; // Fq_w

    P_ = F_x * P_ * F_x.transpose() + Q_ * dt;

}

Mat4 EFK::quaternionProductMatrix(const Quaternion& q, bool left) {
    return left ?
        (Mat4() << q.w(), -q.x(), -q.y(), -q.z(),
                   q.x(),  q.w(), -q.z(),  q.y(),
                   q.y(),  q.z(),  q.w(), -q.x(),
                   q.z(), -q.y(),  q.x(),  q.w()).finished() :
    
        (Mat4() << q.w(), -q.x(), -q.y(), -q.z(),
                   q.x(),  q.w(),  q.z(), -q.y(),
                   q.y(), -q.z(),  q.w(),  q.x(),
                   q.z(),  q.y(), -q.x(),  q.w()).finished();
}

} // namespace
