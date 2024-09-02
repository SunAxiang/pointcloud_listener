// src/main.cpp
#include "pointcloud_converter/pointcloud_converter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_converter");
    PointCloudConverter converter;
    ros::spin();
    return 0;
}
