#include "tracker/tracker_map.h"

namespace track
{
TrackerMap::TrackerMap() {
    // setup known map, UGLY CODE, do this outside...
    const double hw = 85.0f/2;
    vector<Point3d> model_points {
        Point3d(-hw, -hw, 0.0f),
        Point3d( hw, -hw, 0.0f),
        Point3d( hw,  hw, 0.0f),
        Point3d( -hw,  hw, 0.0f)
    };
    for(int i = 0; i < 4; ++i) {
        Point3d p1 = model_points[i];
        Point3d p2 = model_points[(i+1) % 4];
        map_.push_back(SlamLine(p1,p2));
    }
}
void TrackerMap::projectAll(const Vec3& camera_position,
                            const Quaternion& camera_orientation,
                            const Vec4& camera_matrix) {
    for (SlamLine& sl : map_)
        sl.project(camera_position, camera_orientation, camera_matrix);
}

void TrackerMap::project(int s_id,
    const Vec3& camera_position,
    const Quaternion& camera_orientation,
    const Vec4& camera_matrix) {
    map_[s_id].project(camera_position, camera_orientation, camera_matrix);
}

int TrackerMap::getNearest(const Point2d &p, double &best_distance, double threshold, double min_margin) {
    int best_id = -1;
    int best_id2 = -1;
    double best_distance2;
    for (int i = 0; i < map_.size(); ++i) {
        const SlamLine &sl = map_[i];
        if (SlamLine::isAligned(sl, p)) {
            double distance_i = SlamLine::getDistance(map_[i], p);
            if (abs(distance_i) <= threshold) {
                if (best_id == -1 or abs(distance_i) < abs(best_distance)) {
                    // move best to 2nd best
                    best_id2 = best_id;
                    best_id = i;
                    best_distance2 = best_distance;
                    best_distance = distance_i;
                } else if (best_id2 == -1 or abs(distance_i) < abs(best_distance2)) {
                    // if we're not 1st maybe we are 2nd
                    best_id2 = i;
                    best_distance2 = distance_i;
                }
            }
        }
    }
    if (best_id2 == -1) return best_id; // no 2nd, 1st always
    if (abs(best_distance) <= abs(best_distance2) + min_margin) return -2; // not sure about the segment
    return best_id;
}

void TrackerMap::draw2dMap(cv::Mat &img) {
    for (int i = 0; i < map_.size(); ++i) {
        const SlamLine &sl = map_[i];
        cv::Point p1(sl.p1_2d[0], sl.p1_2d[1]);
        cv::Point p2(sl.p2_2d[0], sl.p2_2d[1]);
        cv::line(img, p1, p2, CV_RGB(0,0,255), 1);
        cv::Point p = (p1+p2) * 0.5;
        cv::putText(img, std::to_string(i), p, cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,125,255), 1);
    }
}

void TrackerMap::draw2dMapWithCov(cv::Mat &img, const Eigen::Matrix<double, 7, 7>& P) {
    for (int i = 0; i < map_.size(); ++i) {
        const SlamLine &sl = map_[i];
        cv::Point p1(sl.p1_2d[0], sl.p1_2d[1]);
        cv::Point p2(sl.p2_2d[0], sl.p2_2d[1]);
        cv::line(img, p1, p2, CV_RGB(0,0,255), 1);
        cv::Point p = (p1+p2) * 0.5;
        cv::putText(img, std::to_string(i), p, cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0,125,255), 1);
        // add covariance ellipses,  5.991 is 95% confint
        Eigen::Matrix<double, 2, 7> Fx1 = sl.jac_points_2d_rq.block<2,7>(0,0);
        Eigen::Matrix2d cov1 = Fx1 * P * Fx1.transpose();
	    cv::ellipse(img, getErrorEllipse(5.991, sl.p1_2d, cov1), CV_RGB(0, 255, 255), 1);
        Eigen::Matrix<double, 2, 7> Fx2 = sl.jac_points_2d_rq.block<2,7>(2,0);
        Eigen::Matrix2d cov2 = Fx2 * P * Fx2.transpose();
        cv::ellipse(img, getErrorEllipse(5.991, sl.p2_2d, cov2), CV_RGB(0, 255, 255), 1);
    }
}

cv::RotatedRect TrackerMap::getErrorEllipse(double chisq, const Point2d &mean, const Eigen::Matrix2d& cov) {

    /* adaptation from
        http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
    */
    // get eigenvalues and eigenvectors
    // es.eigenvalues() is column vector sorted increasingly
    // es.eigenvectors() is matrix with eigenvectors as columns
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
    ROS_DEBUG_STREAM("EIGEN VALUES ::: " << es.eigenvalues().transpose());
    // angle between the largest eigenvector and the x-axis
    double angle = atan2(es.eigenvectors().col(1)[1], es.eigenvectors().col(1)[0]);
    // angle between [0,2pi] instaed of [-pi, pi]
    if (angle < 0) angle += M_PI;
    // angle to degrees
    angle *= 180/M_PI;
    // minor and major axes
    double half_major_axis_size = chisq*sqrt(es.eigenvalues()[1]);
    double half_minor_axis_size = chisq*sqrt(es.eigenvalues()[0]);
    if (!(std::isfinite(half_major_axis_size) and std::isfinite(half_minor_axis_size))) {
        half_major_axis_size = half_minor_axis_size = 0;
        ROS_WARN("error ellipse is infinite");
    }
    // return the oriented ellipse (-angle before opencv has cw angles...)
    return cv::RotatedRect(cv::Point2d(mean[0], mean[1]), cv::Size2f(half_major_axis_size, half_minor_axis_size), -angle);
}

}

  