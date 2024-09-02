// src/pointcloud_io.cpp
#include "pointcloud_converter/pointcloud_io.h"
#include <iomanip> // For std::setw, std::setfill
#include <sstream> // For std::stringstream
#include <iostream>

PointCloudIO::PointCloudIO(const std::string &pcl_filename, const std::string &pcl2_filename, const std::string &pcl2_converted_filename)
    : pcl_filename_(pcl_filename), pcl2_filename_(pcl2_filename), pcl2_converted_filename_(pcl2_converted_filename)
{
    file_pcl_.open(pcl_filename_);
    file_pcl2_.open(pcl2_filename_);
    file_pcl2_converted_.open(pcl2_converted_filename_);

    if (!file_pcl_.is_open() || !file_pcl2_.is_open() || !file_pcl2_converted_.is_open())
    {
        throw std::ios_base::failure("Unable to open one or more files.");
    }
}

PointCloudIO::~PointCloudIO()
{
    if (file_pcl_.is_open())
        file_pcl_.close();
    if (file_pcl2_.is_open())
        file_pcl2_.close();
    if (file_pcl2_converted_.is_open())
        file_pcl2_converted_.close();
}

void PointCloudIO::savePointCloudToFile(const sensor_msgs::PointCloud &ros_cloud)
{
    if (file_pcl_.is_open())
    {
        file_pcl_ << "Header:\n  seq: " << ros_cloud.header.seq
                  << "\n  stamp: " << ros_cloud.header.stamp
                  << "\n  frame_id: " << ros_cloud.header.frame_id << "\n";

        file_pcl_ << "Points:\n";
        for (const auto &point : ros_cloud.points)
        {
            file_pcl_ << "  - x: " << point.x << ", y: " << point.y << ", z: " << point.z << "\n";
        }
        file_pcl_ << "\n";
    }
}

void PointCloudIO::savePointCloud2ToFile(const sensor_msgs::PointCloud2 &msg)
{
    if (file_pcl2_.is_open())
    {
        file_pcl2_ << "Header:\n  seq: " << msg.header.seq
                   << "\n  stamp: " << msg.header.stamp
                   << "\n  frame_id: " << msg.header.frame_id << "\n";
        file_pcl2_ << "Height: " << msg.height << "\n";
        file_pcl2_ << "Width: " << msg.width << "\n";

        file_pcl2_ << "Fields:\n";
        for (const auto &field : msg.fields)
        {
            file_pcl2_ << "  - name: " << field.name << "\n";
            file_pcl2_ << "    offset: " << field.offset << "\n";
            file_pcl2_ << "    datatype: " << static_cast<int>(field.datatype) << "\n";
            file_pcl2_ << "    count: " << field.count << "\n";
        }

        file_pcl2_ << "Is Big Endian: " << (msg.is_bigendian ? "true" : "false") << "\n";
        file_pcl2_ << "Point Step: " << msg.point_step << "\n";
        file_pcl2_ << "Row Step: " << msg.row_step << "\n";
        file_pcl2_ << "Is Dense: " << (msg.is_dense ? "true" : "false") << "\n";

        file_pcl2_ << "Data (hex):\n";
        // Binary data output in hexadecimal format
        for (const auto &byte : msg.data)
        {
            file_pcl2_ << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        file_pcl2_ << std::dec << "\n"; // Restore to decimal format

        file_pcl2_ << "\n";
    }
}
