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

} // namespace
