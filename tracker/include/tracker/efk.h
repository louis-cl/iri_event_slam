#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

using Vec3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;
using Mat13 = Eigen::Matrix<double, 13, 13>;
using Mat4 = Eigen::Matrix<double, 4, 4>;

namespace track {

class EFK {
public:
    // STATE DEFINITION
    struct State {
        Vec3 r;       // position          [x,y,z]        cartesian
        Quaternion q; // orientation       {w,x,y,z}      quaternion
        Vec3 v;       // linear velocity   [vx,vy,vz]     cartesian 
        AngleAxis w;  // angular velocity  theta*u        angle-axis (radians, cartesian)
    };
    
    EFK();
    EFK(const Vec3& sigma_v, const Vec3& sigma_w, double sigma_d);

    // initialize state
    void init(const EFK::State& X0);
    // predict the next state after dt seconds
    void predict(double dt);
    //void correct(double dist);


    Mat13 Q_; // motion uncertainty per second
private:
    State X_;  // state
    Mat13 P_; // state covariance in order [r q v w]

    // UNCERTAINTY CONSTANTS
    double R_; // measurement noise

    // q1 . q2 = [q2]r * q1  = [q1]l * q2
    // returns [q]l if left else [q]r
    Mat4 quaternionProductMatrix(const Quaternion& q, bool left = true);
};

} // namespace