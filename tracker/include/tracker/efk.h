#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Quaternion = Eigen::Quaterniond;
using AngleAxis = Eigen::AngleAxisd;
using Mat13 = Eigen::Matrix<double, 13, 13>;
using Mat4 = Eigen::Matrix<double, 4, 4>;

using std::sin;
using std::cos;
using std::round;

namespace track {

class EFK {
// Extended Kalman Filter implementation for a camera state
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
    void init(const EFK::State& X0, const Mat13& = Mat13::Zero());
    // predict the next state after dt seconds
    void predict(double dt);
    // update state after distance measurement
    void update(double dist, const Eigen::Matrix<double, 1, 7>& H);
    
    // get current state
    EFK::State getState();
    Mat13 getCovariance();
    
//private:
    State X_;  // state
    Mat13 P_; // state covariance in order [r v q w]

    // UNCERTAINTY CONSTANTS
    Mat13 Q_; // motion uncertainty per second
    double R_; // measurement noise

    // q1 . q2 = [q2]r * q1  = [q1]l * q2
    // returns [q]l if left else [q]r
    // variables order is w,x,y,z
    Mat4 quaternionProductMatrix(const Quaternion& q, bool left = true);
};

} // namespace