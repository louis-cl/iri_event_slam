# ROS DAVIS Tracker
You first need to install `davis_ros_driver` from https://github.com/uzh-rpg/rpg_dvs_ros

### ROS workspace
You need a ROS workspace. If you followed the instructions above, this workspace is `catkin_ws`.

You need to source the setup file:
```sh
cd catkin_ws
source devel/setup.bash
```

### Building
```sh
catkin build track_init tracker
```
### Running
Before running you need to have the davis camera calibrated with no tangential distortion http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
#### Easy way
```sh
    roslaunch tracker track.launch
```
or a nodelet version
```sh
    roslaunch tracker track_nodelet.launch
```
Then start tracking with `rostopic pub /reset std_msgs/Bool "data: true" -1`, you can also use it to reset the tracker (starts from the last pose received) in case it gets lost.

Those are running 5 nodes: davis_ros_driver, track_init, tracker and a couple image_view for visualization. You can run them individually, check the launch files for more details.

#### With a ros bag
This is useful for debugging or slow step-by-step visualization.
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

### ROS Nodes
#### track_init
Computes a camera pose (`geometry_msgs/PoseStamped`) from an image feed (known map)
- Publications: 
    * pose [geometry_msgs/PoseStamped]: computed camera pose relative to the map
    * rendering [sensor_msgs/Image]: visualization of found map + reprojected map vertices
- Subscriptions: 
    * camera_info [sensor_msgs/CameraInfo]: camera parameters
    * image [sensor_msgs/Image]: raw image feed

#### tracker
Tracks a segment map with events from the camera
- Publications: 
    * /map_events [sensor_msgs/Image]: visualization of the tracked map with events (used in red)
    * /tracked_pose [geometry_msgs/PoseStamped]: estimated camera pose
- Subscriptions: 
    * /camera_info [sensor_msgs::CameraInfo]: camera parameters
    * /camera_pose [geometry_msgs::PoseStamped]: first camera pose (usually from track_init)
    * /events [dvs_msgs::EventArray]: camera events
    * /reset [std_msgs::Bool]: start&reset flag channel, sending a msgs starts tracking or resets it

### Files
    ├── README.md
    ├── tracker
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── tracker
    │   │       ├── efk.h
    │   │       ├── slam_line.h
    │   │       ├── tracker.h
    │   │       ├── tracker_map.h
    │   │       └── tracker_nodelet.h
    │   ├── launch
    │   │   ├── track.launch
    │   │   └── track_nodelet.launch
    │   ├── package.xml
    │   ├── src
    │   │   ├── efk.cpp
    │   │   ├── slam_line.cpp
    │   │   ├── tracker.cpp
    │   │   ├── tracker_map.cpp
    │   │   ├── tracker_node.cpp
    │   │   └── tracker_nodelet.cpp
    │   ├── test
    │   │   └── test.cpp
    │   └── tracker_nodelet.xml
    └── track_init
        ├── CMakeLists.txt
        ├── include
        │   └── track_init
        │       └── track_init.h
        ├── launch
        │   └── track_init.launch
        ├── package.xml
        └── src
            ├── track_init.cpp
            ├── track_init_node.cpp
            └── track_test.cpp
