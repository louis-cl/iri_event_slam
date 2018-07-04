#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

using Vec3  = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;
using Mat13 = Eigen::Matrix<double, 13, 13>;

namespace track {

class EFK {
public:
    // STATE DEFINITION
    struct State {
        Vec3 r;       // position          x,y,z          cartesian
        Quaternion q; // orientation       x,y,z,w        quaternion
        Vec3 v;       // linear velocity   vx,vy,vz       cartesian 
        Vec3 w;       // angular velocity  theta*u        angle-axis (radians, cartesian)
    };
    
    EFK();
    EFK(const Vec3& sigma_v, const Vec3& sigma_w, double sigma_d);

    void init(const EFK::State& X0);
    //void predict(double dt);
    //void correct(double dist);


    Mat13 Q_; // motion uncertainty per second
private:
    State X_;  // state
    Mat13 P_; // state covariance

    // UNCERTAINTY CONSTANTS
    double R_; // measurement noise
};

} // namespace