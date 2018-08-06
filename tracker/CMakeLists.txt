cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME tracker)
project(${PROJECT_NAME})

find_package(catkin_simple REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

cs_add_executable(tracker
  src/tracker.cpp
  src/tracker_nodelet.cpp
  src/tracker_node.cpp
  src/tracker_map.cpp
  src/efk.cpp
  src/slam_line.cpp
)

# nodelet into library
cs_add_library(tracker_nodelet
  src/tracker.cpp
  src/tracker_nodelet.cpp
  src/tracker_node.cpp
  src/tracker_map.cpp
  src/efk.cpp
  src/slam_line.cpp
)

target_link_libraries(tracker
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${GTEST_LIBRARIES}
   pthread
)

target_link_libraries(tracker_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${GTEST_LIBRARIES}
   pthread
)

# add test suite
catkin_add_gtest(tracker-test
  test/test.cpp
  src/slam_line.cpp  
)
target_link_libraries(tracker-test ${catkin_LIBRARIES})

cs_install()

install(FILES tracker_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)