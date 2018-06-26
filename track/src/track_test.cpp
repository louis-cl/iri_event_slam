#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    const geometry_msgs::Vector3& w = msg->angular_velocity;
    const geometry_msgs::Vector3& v = msg->linear_acceleration;
    ROS_INFO("\n\tAngular velocity: [%f,%f,%f]\n"
             "\tLinear acceleration: [%f,%f,%f]\n", w.x, w.y, w.z, v.x, v.y, v.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_test");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/dvs/imu", 1000, IMUCallback);

    ros::spin();
    return 0;
}
