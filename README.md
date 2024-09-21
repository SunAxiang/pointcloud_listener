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


pointcloud_converter.cpp：手动转换；除了 `x`, `y`, `z` 坐标外，其他字段 fields 会被存储在 PointCloud 的通道中（channels）。

sensor_msgs::PointCloud2 => sensor_msgs::PointCloud


pointcloud_converter2.cpp 使用:sensor_msgs::convertPointCloud2ToPointCloud 函数进行转换

sensor_msgs::PointCloud2 => sensor_msgs::PointCloud

---

pointcloud_processor.cpp：手动转换；除了 `x`, `y`, `z` 坐标外，其他 通道 channels 会被存储在 `PointCloud`2 的字段 fields 中。

sensor_msgs::PointCloud => sensor_msgs::PointCloud2


pointcloud_processor2.cpp:使用 sensor_msgs::convertPointCloudToPointCloud2 函数进行转换

sensor_msgs::PointCloud => sensor_msgs::PointCloud2

---

pointcloud_converter => snail-radar => 20231208 => data4.bag => /ars548

pointcloud_processor => xxx => xxx => xxx => xxx

---
