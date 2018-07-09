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

void EFK::init(const State& X0, const Mat13& P0) {
    X_ = X0;
    P_ = P0;
}

void EFK::predict(double dt) {
    /* constant velocity model
        r = r + v * dt
        q = q . Quaternion(w*dt)
        v = v
        w = w
    */
    Quaternion q = X_.q; // store old orientation
    // should I normalize to prevents my quaternion to blow up ?
    // normalizing probably implies changing P (scale change) q = q/|q| 

    // update state X_
    X_.r += X_.v * dt;
    
    Quaternion qw;
    qw = AngleAxis(X_.w.angle()*dt, X_.w.axis());
    X_.q *= qw;

    // update covariance P_
    /*
    F_x =   1       0       dt      0 
            0       Fq_q    0       Fq_w
            0       0       1       0
            0       0       0       1
    */

    Mat13 F_x(Mat13::Identity());
    // Fr_v
    F_x.block<3,3>(0,7).diagonal().fill(dt);
    // Fq_q
    F_x.block<4,4>(3,3) = quaternionProductMatrix(qw, false);
    // Fq_w 
    Vec3 u = X_.w.axis();
    double theta = X_.w.angle() * dt;
    /* q . Quaternion(w*dt) => Fq_w = [q]l * JacQuaternion(w*dt)_w
       w*dt = u*theta  with |u| = 1
       JacQuaternion(w*dt)_w = JacQuaternion(w*dt)_u * JacU_w +
                               JacQuaternion(w*dt)_theta * JacTheta_w

       [ 0   0   0 ]                         [ -sin(theta/2)  ]
       [ 1   0   0 ]                         [                ]
       [ 0   1   0 ] * dt/theta   +   dt/2 * [ cos(theta/2)*u ] * u.transpose()
       [ 0   0   1 ]                         [                ]


       if |w*dt| = theta is small, we have a bad jacobian => aproximate with Taylor
       JacQuaternion(w*dt)_w = JacQuaternion(w*dt)_(w*dt) * Jac(w*dt)_w

       [ -dt/4 * w.transpose() ] 
       [ 1/2 * Identity3       ] *  (dt * Identity3)
    */                               
    Eigen::Matrix<double, 4, 3> JacQuaternion_w;
    if (theta >= 1e-6) {
        JacQuaternion_w = dt/2 *
            (Vec4() << -sin(theta/2), cos(theta/2)*u).finished() * u.transpose();
        JacQuaternion_w.block<3,3>(1,0).diagonal().array() += 1/X_.w.angle(); // dt/theta
    } else {
        JacQuaternion_w << -dt/2 * X_.w.axis(), 1,0,0, 0,1,0, 0,0,1;
        JacQuaternion_w *= dt/2;
    }
    F_x.block<4,3>(3,10) = quaternionProductMatrix(q) * JacQuaternion_w;

    P_ = F_x * P_ * F_x.transpose() + Q_ * dt;

    /*
        This could be improved by reordering variables r,q,v,w -> r,v,q,w
        
        F_x is then block diagonal :  F1 0 ; 0 F2
        F1 = 1 dt; 0 1   and    F2 = Fq_q Fq_w; 0 1
        
        P update can be simplified:
        P by blocks P1 P2; P2' P3 then
            P1 = F1*P1*F1'
            P2 = F1*P2*F2'
            P3 = F2*P3*F2'
    */
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
