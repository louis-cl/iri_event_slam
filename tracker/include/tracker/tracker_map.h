#pragma once
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>
#include "slam_line.h"

using std::vector;
using std::abs;
using std::atan2;
using std::sqrt;

using Point2d = Eigen::Vector2d;
using Point3d = Eigen::Vector3d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Quaternion = Eigen::Quaterniond;

namespace track
{
class TrackerMap {
    public:
        TrackerMap();
        
        // project all 3d segment to the 2d map
        void projectAll(const Vec3& camera_position,
                        const Quaternion& camera_orientation,
                        const Vec4& camera_matrix);
        
        // project a 3d segment to the 2d map
        // camera_matrix is [u0 u1 fx fy]
        void project(int s_id,
            const Vec3& camera_position,
            const Quaternion& camera_orientation,
            const Vec4& camera_matrix);

        // get distance of a point to a segment in the 2d map
        inline double getDistance(const Point2d &p, int s_id) {
            return SlamLine::getDistance(map_[s_id], p);
        }
        // same as above + jacobians with respect to r,q
        inline double getDistance(const Point2d& p, int s_id,
                Eigen::RowVector3d& jac_d_r, Eigen::RowVector4d& jac_d_q) {
            return SlamLine::getDistance(map_[s_id], p, jac_d_r, jac_d_q);
        }

        int getNearest(const Point2d &p, double &best_distance, double threshold, double min_margin);

        // draw the 2d map segments in green
        void draw2dMap(cv::Mat &img);
        // draw the 2d map segments in green with their cov ellipse
        void draw2dMapWithCov(cv::Mat &img, const Eigen::Matrix<double, 7, 7>& P);

    private:
        vector<SlamLine> map_;

        cv::RotatedRect getErrorEllipse(double chisq, const Point2d &mean, const Eigen::Matrix2d& cov);
};

}
