catkin_make -j6

source devel/setup.bash

rosrun pointcloud_listener pointcloud_converter

rosrun pointcloud_listener pointcloud_processor

rosrun pointcloud_listener pointcloud_processor2

rosrun pointcloud_listener pointcloud_listener.py

roslaunch pointcloud_listener pointcloud_listener.launch

*---*

*pointcloud_converter.cpp：sensor_msgs::PointCloud2=>sensor_msgs::PointCloud；*

*pointcloud_converter2.cpp:自定义的点云数据类型（custom_point_types.h）使用	pcl::fromROSMsg(*msg, cloud)进行转换。*

*pointcloud_processor.cpp：sensor_msgs::PointCloud=>sensor_msgs::PointCloud2；*

*pointcloud_processor2.cpp:自定义的点云数据类型（custom_point_types.h）使用xxx进行转换（还没有完成）。*
