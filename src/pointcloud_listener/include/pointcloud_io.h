// include/pointcloud_converter/pointcloud_io.h
#ifndef POINTCLOUD_IO_H
#define POINTCLOUD_IO_H

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <fstream>

class PointCloudIO
{
public:
    // 构造函数
    PointCloudIO(const std::string &pcl_filename, const std::string &pcl2_filename, const std::string &pcl2_converted_filename);

    // 析构函数
    ~PointCloudIO();

    // 保存 PointCloud 数据到文件
    void savePointCloudToFile(const sensor_msgs::PointCloud &ros_cloud);

    // 保存 PointCloud2 数据到文件
    void savePointCloud2ToFile(const sensor_msgs::PointCloud2 &msg);

private:
    // 文件流
    std::ofstream file_pcl_;
    std::ofstream file_pcl2_;
    std::ofstream file_pcl2_converted_;

    // 文件路径
    std::string pcl_filename_;
    std::string pcl2_filename_;
    std::string pcl2_converted_filename_;
};

#endif // POINTCLOUD_IO_H
