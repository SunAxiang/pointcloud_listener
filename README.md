catkin_make -j6


source devel/setup.bash

rosrun pointcloud_listener pointcloud_converter

rosrun pointcloud_listener pointcloud_processor

rosrun pointcloud_listener pointcloud_processor2


rosrun pointcloud_listener pointcloud_listener.py

roslaunch pointcloud_listener pointcloud_listener.launch

