#include "custom_point_type_converter.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <atomic>

class PointCloudConverter
{
public:
    PointCloudConverter() : message_count_(0)
    {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("/ars548", 10, &PointCloudConverter::callback, this);

        // Create a directory named "txt"
        if (mkdir("txt", 0777) && errno != EEXIST)
        {
            ROS_ERROR("Unable to create 'txt' directory");
        }

        // Open the output files for writing
        pointcloud2_file_.open("txt/converter_pointcloud2.txt");
        converted_file_.open("txt/converter_pointcloud.txt");

        if (!pointcloud2_file_.is_open() || !converted_file_.is_open())
        {
            ROS_ERROR("Unable to open output files for writing");
        }

        // Start a separate thread to monitor the timeout
        monitor_thread_ = std::thread(&PointCloudConverter::monitorTimeout, this);
    }

    ~PointCloudConverter()
    {
        if (monitor_thread_.joinable())
        {
            monitor_thread_.join();
        }

        if (pointcloud2_file_.is_open())
        {
            pointcloud2_file_.close();
        }

        if (converted_file_.is_open())
        {
            converted_file_.close();
        }
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (message_count_ < 3)
        {
            // Save the raw PointCloud2 data to a txt file
            savePointCloud2ToFile(*msg, pointcloud2_file_);

            // Convert PointCloud2 to custom PointCloud type
            pcl::PointCloud<CustomPointType> cloud;
            pcl::fromROSMsg(*msg, cloud);

            // Save the converted PointCloud data to a txt file
            savePointCloudToFile(cloud, converted_file_);

            ROS_INFO("Saved PointCloud2 data and converted PointCloud data");

            message_count_++;
        }

        // If three messages have been received, shut down the node
        if (message_count_ >= 3)
        {
            ros::shutdown();
        }
    }

private:
    ros::Subscriber sub_;
    std::ofstream pointcloud2_file_;
    std::ofstream converted_file_;
    std::atomic<int> message_count_;
    std::thread monitor_thread_;

    // Monitor the timeout for receiving messages
    void monitorTimeout()
    {
        ros::Duration timeout(5.0);
        timeout.sleep();

        if (message_count_ < 3)
        {
            ROS_WARN("Timeout reached before receiving three messages. Shutting down.");
            ros::shutdown();
        }
    }

    // Save PointCloud2 data to a TXT file
    // Save PointCloud2 data to a TXT file
    void savePointCloud2ToFile(const sensor_msgs::PointCloud2 &msg, std::ofstream &file)
    {
        if (file.is_open())
        {
            // Write message number
            file << "Message " << message_count_ + 1 << ":\n";

            // Write header information
            file << "Header:\n";
            file << "  seq: " << msg.header.seq << "\n";
            file << "  stamp: " << msg.header.stamp << "\n";
            file << "  frame_id: " << msg.header.frame_id << "\n";

            // Write basic PointCloud2 information
            file << "Height: " << msg.height << "\n";
            file << "Width: " << msg.width << "\n";

            // Write field information
            file << "Fields:\n";
            for (const auto &field : msg.fields)
            {
                file << "  - name: " << field.name << "\n";
                file << "    offset: " << field.offset << "\n";
                file << "    datatype: " << static_cast<int>(field.datatype) << "\n";
                file << "    count: " << field.count << "\n";
            }

            file << "Is Big Endian: " << (msg.is_bigendian ? "true" : "false") << "\n";
            file << "Point Step: " << msg.point_step << "\n";
            file << "Row Step: " << msg.row_step << "\n";
            file << "Is Dense: " << (msg.is_dense ? "true" : "false") << "\n";

            // Write point cloud data (hexadecimal representation)
            file << "Data (hex):\n";
            for (const auto &byte : msg.data)
            {
                file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            file << "\n\n"; // Add a new line after each message
        }
        else
        {
            ROS_ERROR("Unable to write to PointCloud2 file");
        }
    }

    // Save converted PointCloud data to a TXT file
    // Save converted PointCloud data to a TXT file
    // Save converted PointCloud data to a TXT file
    void savePointCloudToFile(const pcl::PointCloud<CustomPointType> &cloud, std::ofstream &file)
    {
        if (file.is_open())
        {
            // Write header information
            file << "Message " << message_count_ + 1 << ":\n";
            file << "Header:\n";
            file << "  seq: " << cloud.header.seq << "\n";
            file << "  stamp: " << cloud.header.stamp << "\n";
            file << "  frame_id: " << cloud.header.frame_id << "\n";

            // Write points
            file << "Points:\n";
            for (const auto &point : cloud.points)
            {
                file << "  - x: " << point.x << ", y: " << point.y << ", z: " << point.z << "\n";
            }

            // Write channels
            file << "Channels:\n";

            file << "  - name: Doppler\n";
            file << "    values:\n";
            for (const auto &point : cloud.points)
            {
                file << "      - " << point.doppler << "\n";
            }

            file << "  - name: Intensity\n";
            file << "    values:\n";
            for (const auto &point : cloud.points)
            {
                file << "      - " << point.intensity << "\n";
            }

            file << "  - name: Range Std\n";
            file << "    values:\n";
            for (const auto &point : cloud.points)
            {
                file << "      - " << point.range_std << "\n";
            }

            file << "  - name: Azimuth Std\n";
            file << "    values:\n";
            for (const auto &point : cloud.points)
            {
                file << "      - " << point.azimuth_std << "\n";
            }

            file << "  - name: Elevation Std\n";
            file << "    values:\n";
            for (const auto &point : cloud.points)
            {
                file << "      - " << point.elevation_std << "\n";
            }

            file << "  - name: Doppler Std\n";
            file << "    values:\n";
            for (const auto &point : cloud.points)
            {
                file << "      - " << point.doppler_std << "\n";
            }

            file << "\n"; // Add a new line after each message
        }
        else
        {
            ROS_ERROR("Unable to write to converted PointCloud file");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_converter");
    PointCloudConverter converter;
    ros::spin();
    return 0;
}
