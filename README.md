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


_pointcloud_converter.cpp：手动转换；除了 `x`, `y`, `z` 坐标外，其他字段fields会被存储在 `PointCloud` 的通道中（`channels`）。_

sensor_msgs::PointCloud2 => sensor_msgs::PointCloud

pointcloud_converter2.cpp 使用:sensor_msgs::convertPointCloud2ToPointCloud函数

sensor_msgs::PointCloud2 => sensor_msgs::PointCloud

---

_pointcloud_processor.cpp：手动转换；除了 `x`, `y`, `z` 坐标外，其他 `通道channels`会被存储在 `PointCloud`2 的字段fields中。_

_sensor_msgs::PointCloud => sensor_msgs::PointCloud2_

_pointcloud_processor2.cpp:使用sensor_msgs::convertPointCloudToPointCloud2函数_

_sensor_msgs::PointCloud => sensor_msgs::PointCloud2_

---

pointcloud_converter  => snail-radar => 20231208 => data4.bag => /ars548

pointcloud_processor  => xxx =>  xxx => xxx => xxx

---
