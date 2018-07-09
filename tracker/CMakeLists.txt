cmake_minimum_required(VERSION 2.8.3)
project(tracker)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

cs_add_executable(tracker
  src/tracker.cpp
  src/tracker_node.cpp
  src/tracker_map.cpp
  src/efk.cpp
)

target_link_libraries(tracker
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
 

cs_install()