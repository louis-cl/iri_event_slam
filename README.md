# ROS DAVIS Tracker
You need to install `davis_ros_driver` first from https://github.com/uzh-rpg/rpg_dvs_ros
### Building
```sh
catkin build track_init tracker
```
### Running
In three different consoles:
```sh
$ roscore
$ roslaunch track_init track_init.launch
$ rosrun tracker tracker camera_info:=/dvs/camera_info events:=/dvs/events camera_pose:=/pose
```
---
If you have a bag data with:
* /dvs/camera_info	: sensor_msgs/CameraInfo
* /dvs/events           : dvs_msgs/EventArray
* /track/init_pose      : geometry_msgs/PoseStamped
* /track/init_rendering : sensor_msgs/Image (optional)

start ros
`roscore`
and the tracker
`rosrun tracker tracker camera_info:=/dvs/camera_info camera_pose:=/track/init_pose events:=/dvs/events`
play the bag in pause mode
`rosbag play track_data.bag --pause`
press `s` multiple time in the rosbag terminal until the tracker gets the `camera_info` and `camera_pose`
send a reset event to the tracker to start it
`rostopic pub /reset std_msgs/Bool "data: true" -1`
return to the rosbag and keep pressing `s` (step by step) or `space` (play/pause)
---
You can also visualize the **track_init** square detection and the projected map with events:
```sh
rosrun image_view image_view image:=/track/init_rendering
rosrun image_view image_view image:=/map_events
```
