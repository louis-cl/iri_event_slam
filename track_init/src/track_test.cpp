#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"

#include <cmath>

ros::Publisher poseStampedPub;

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    const geometry_msgs::Vector3& w = msg->angular_velocity;
    const geometry_msgs::Vector3& v = msg->linear_acceleration;
    ROS_INFO("\n\tAngular velocity: [%f,%f,%f]\n"
             "\tLinear acceleration: [%f,%f,%f]\n", w.x, w.y, w.z, v.x, v.y, v.z);

    geometry_msgs::PoseStamped poseStamped;

    poseStamped.header.frame_id="map";
    poseStamped.header.stamp = ros::Time::now();

    poseStamped.pose.position.x = v.x;
    poseStamped.pose.position.y = v.y;
    poseStamped.pose.position.z = v.z;

    double angle = sqrt(w.x*w.x + w.y*w.y + w.z*w.z);

    if (angle > 0.0) {
        poseStamped.pose.orientation.x = w.x * sin(angle/2)/angle;
        poseStamped.pose.orientation.y = w.y * sin(angle/2)/angle;
        poseStamped.pose.orientation.z = w.z * sin(angle/2)/angle;
        poseStamped.pose.orientation.w = cos(angle/2);
    } else {
        poseStamped.pose.orientation.x = 0;
        poseStamped.pose.orientation.y = 0;
        poseStamped.pose.orientation.z = 0;
        poseStamped.pose.orientation.w = 1;
    }

    

    poseStampedPub.publish(poseStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_test");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/dvs/imu", 1000, IMUCallback);
    poseStampedPub = nh.advertise<geometry_msgs::PoseStamped>("pose", 2, true);

    ros::spin();
    return 0;
}
