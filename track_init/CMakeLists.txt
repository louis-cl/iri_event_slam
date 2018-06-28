cmake_minimum_required(VERSION 2.8.3)
project(track_init)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
# cs_add_executable(track_test
#   src/track_test.cpp
# )

cs_add_executable(track_init
  src/track_init.cpp
  src/track_init_node.cpp
)

# link the executable to the necesarry libs
# target_link_libraries(track_test
#    ${catkin_LIBRARIES}
# )

target_link_libraries(track_init
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
 

cs_install()