#include <gtest/gtest.h>
#include "tracker/slam_line.h"
#include <iostream>
#include <cmath>

using namespace track;
using namespace std;

TEST(Slamline, Constructor){
    Point3d p1(1,0,1);
    Point3d p2(2,0,1);
    SlamLine sl = SlamLine(p1, p2);
    EXPECT_EQ(p1, sl.p1_3d);
    EXPECT_EQ(p2, sl.p2_3d);
}

TEST(Slamline, CameraFrameIsWorldFrame){
    Point3d p1(1,0,1);
    Point3d p2(4,0,2);
    SlamLine sl = SlamLine(p1, p2);
    // world frame = camera frame
    sl.project(Vec3(0,0,0), Quaternion(1,0,0,0), Vec4(1,1,0,0));
    EXPECT_EQ(Point2d(1,0), sl.p1_2d);
    EXPECT_EQ(Point2d(2,0), sl.p2_2d);
}

TEST(Slamline, CameraFrameIsAlignedWithWorldFrame){
    Point3d p1(1,0,1);
    Point3d p2(3,0,1);
    SlamLine sl = SlamLine(p1, p2);
    // world frame = camera frame
    sl.project(Vec3(1,0,0), Quaternion(1,0,0,0), Vec4(1,1,0,0));
    EXPECT_EQ(Point2d(0,0), sl.p1_2d);
    EXPECT_EQ(Point2d(2,0), sl.p2_2d);
}

TEST(Slamline, CameraFrameIsAtWorldFrame){
    Point3d p1(1,1,1);
    Point3d p2(3,1,1);
    SlamLine sl = SlamLine(p1, p2);
    // camera frame at 0,0,0 rotated 90deg around X axis
    sl.project(Vec3(0,0,0), Quaternion(cos(M_PI/4),sin(M_PI/4),0,0), Vec4(1,1,0,0));
    EXPECT_DOUBLE_EQ(-1, sl.p1_2d[0]);
    EXPECT_DOUBLE_EQ(-1, sl.p1_2d[1]);
    EXPECT_DOUBLE_EQ(-3, sl.p2_2d[0]);
    EXPECT_DOUBLE_EQ(-1, sl.p2_2d[1]);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
