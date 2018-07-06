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