catkin_make -j6

source devel/setup.bash

---

rosrun pointcloud_listener pointcloud_converter

rosrun pointcloud_listener pointcloud_converter2

---

rosrun pointcloud_listener pointcloud_processor

rosrun pointcloud_listener pointcloud_processor2

---

rosrun pointcloud_listener pointcloud_listener.py

roslaunch pointcloud_listener pointcloud_listener.launch

_---_

_pointcloud_converter.cpp：不使用自定义数据类型；除了 `x`, `y`, `z` 坐标外，其他字段会被存储在 `PointCloud` 的通道中（`channels`）。_

sensor_msgs::PointCloud2 => sensor_msgs::PointCloud

pointcloud_converter2.cpp 使用:自定义的点云数据类型（custom_point_types.h）；使用 pcl::fromROSMsg(\*msg, cloud)进行转换。

sensor_msgs::PointCloud2=>pcl::PointCloud `<CustomPointConverterType>` => sensor_msgs::PointCloud

---

_pointcloud_processor.cpp：不使用自定义数据类型；_

_sensor_msgs::PointCloud=>pcl::PointCloud=>pcl::PointCloud2=>sensor_msgs::PointCloud2_

_pointcloud_processor2.cpp:使用自定义的点云数据类型（custom_point_types.h）；使用 xxx 进行转换（还没有完成）。_

---

sensor_msgs/PointCloud2  => snail-radar => 20231208 => data4.bag => /ars548

sensor_msgs/PointCloud  => xxx =>  xxx => xxx => xxx

---
