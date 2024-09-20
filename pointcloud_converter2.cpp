#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <atomic>
#include "custom_point_type_converter.h" // 包含自定义点云类型头文件

class PointCloudConverter
{
public:
    PointCloudConverter() : message_count_(0)
    {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("/ars548", 10, &PointCloudConverter::callback, this);

        // 创建目录 "txt"
        if (mkdir("txt", 0777) && errno != EEXIST)
        {
            ROS_ERROR("Unable to create 'txt' directory");
        }

        // 打开输出文件
        pointcloud2_file_.open("txt/converter_pointcloud2.txt");
        converted_file_.open("txt/converter_pointcloud.txt");

        if (!pointcloud2_file_.is_open() || !converted_file_.is_open())
        {
            ROS_ERROR("Unable to open output files for writing");
        }

        // 启动一个线程监视超时
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
            // 保存原始 PointCloud2 数据到文件
            savePointCloud2ToFile(*msg, pointcloud2_file_);

            // 定义一个 PCL 点云对象，使用自定义点云类型
            pcl::PointCloud<CustomPointConverterType>::Ptr cloud(new pcl::PointCloud<CustomPointConverterType>);

            // 将 PointCloud2 转换为自定义点云类型
            pcl::fromROSMsg(*msg, *cloud);

            // 保存转换后的自定义点云数据到文件
            saveCustomPointCloudToFile(cloud, converted_file_);

            // 将自定义点云转换为 sensor_msgs::PointCloud
            sensor_msgs::PointCloud sensor_cloud;
            customPointCloudToSensorPointCloud(cloud, sensor_cloud);

            // 输出转换结果
            ROS_INFO("Converted custom point cloud to sensor_msgs::PointCloud");

            message_count_++;
        }

        // 如果接收到三个消息，则关闭节点
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

    // 监视接收消息的超时时间
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

    // 保存 PointCloud2 数据到文件
    void savePointCloud2ToFile(const sensor_msgs::PointCloud2 &msg, std::ofstream &file)
    {
        if (file.is_open())
        {
            file << "Message " << message_count_ + 1 << ":\n";

            // 写入头信息
            file << "Header:\n";
            file << "  seq: " << msg.header.seq << "\n";
            file << "  stamp: " << msg.header.stamp << "\n";
            file << "  frame_id: " << msg.header.frame_id << "\n";

            // 写入 PointCloud2 的基本信息
            file << "Height: " << msg.height << "\n";
            file << "Width: " << msg.width << "\n";

            // 写入字段信息
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

            // 写入点云数据（十六进制表示）
            file << "Data (hex):\n";
            for (const auto &byte : msg.data)
            {
                file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            file << "\n\n";
        }
        else
        {
            ROS_ERROR("Unable to write to PointCloud2 file");
        }
    }

    // 保存转换后的自定义点云数据到文件
    void saveCustomPointCloudToFile(const pcl::PointCloud<CustomPointConverterType>::Ptr &cloud, std::ofstream &file)
    {
        if (file.is_open())
        {
            file << "Message " << message_count_ + 1 << ":\n";
            file << "Points:\n";
            for (const auto &point : cloud->points)
            {
                file << "  - x: " << point.x << ", y: " << point.y << ", z: " << point.z << "\n";
                file << "    doppler: " << point.doppler << ", intensity: " << point.intensity << "\n";
                file << "    range_std: " << point.range_std << ", azimuth_std: " << point.azimuth_std << "\n";
                file << "    elevation_std: " << point.elevation_std << ", doppler_std: " << point.doppler_std << "\n";
            }
            file << "\n";
        }
        else
        {
            ROS_ERROR("Unable to write to converted custom PointCloud file");
        }
    }

    // 自定义点云转换为 sensor_msgs::PointCloud
    void customPointCloudToSensorPointCloud(const pcl::PointCloud<CustomPointConverterType>::Ptr &custom_cloud,
                                            sensor_msgs::PointCloud &sensor_cloud)
    {
        // 设置 header
        sensor_cloud.header.stamp = ros::Time::now();
        sensor_cloud.header.frame_id = "custom_frame";

        // 将自定义点云的 x, y, z 拷贝到 sensor_msgs::PointCloud 中
        sensor_cloud.points.resize(custom_cloud->size());
        for (size_t i = 0; i < custom_cloud->points.size(); ++i)
        {
            sensor_cloud.points[i].x = custom_cloud->points[i].x;
            sensor_cloud.points[i].y = custom_cloud->points[i].y;
            sensor_cloud.points[i].z = custom_cloud->points[i].z;
        }

        // 创建并填充通道数据
        sensor_msgs::ChannelFloat32 doppler_channel;
        doppler_channel.name = "doppler";
        sensor_msgs::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensity";

        doppler_channel.values.resize(custom_cloud->size());
        intensity_channel.values.resize(custom_cloud->size());

        for (size_t i = 0; i < custom_cloud->points.size(); ++i)
        {
            doppler_channel.values[i] = custom_cloud->points[i].doppler;
            intensity_channel.values[i] = custom_cloud->points[i].intensity;
        }

        sensor_cloud.channels.push_back(doppler_channel);
        sensor_cloud.channels.push_back(intensity_channel);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_converter");
    PointCloudConverter converter;
    ros::spin();
    return 0;
}
