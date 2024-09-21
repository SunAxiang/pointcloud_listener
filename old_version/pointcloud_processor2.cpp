#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <atomic>
#include "custom_point_type_processor.h"

class PointCloudHandler
{
public:
    PointCloudHandler() : message_count_(0), shutdown_requested_(false)
    {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("/radar_enhanced_pcl", 10, &PointCloudHandler::callback, this);

        // 创建 "txt" 目录
        if (!createDirectory("txt"))
        {
            ROS_ERROR("Failed to create 'txt' directory.");
        }

        // 打开输出文件
        pointcloud_file_.open("txt/processor2_pointcloud.txt");
        pointcloud2_file_.open("txt/processor2_pointcloud2.txt");

        if (!pointcloud_file_.is_open() || !pointcloud2_file_.is_open())
        {
            ROS_ERROR("Failed to open output files.");
        }

        // 启动监控线程
        monitor_thread_ = std::thread(&PointCloudHandler::monitorTimeout, this);
    }

    ~PointCloudHandler()
    {
        shutdown_requested_ = true; // 设置标志以停止监控线程

        if (monitor_thread_.joinable())
        {
            monitor_thread_.join();
        }

        if (pointcloud_file_.is_open())
        {
            pointcloud_file_.close();
        }

        if (pointcloud2_file_.is_open())
        {
            pointcloud2_file_.close();
        }
    }

    void callback(const sensor_msgs::PointCloud::ConstPtr &msg)
    {
        if (message_count_ < 3)
        {
            // 手动转换 PointCloud 到 PointCloud2
            sensor_msgs::PointCloud2 pcl_cloud2;
            convertPointCloudToPointCloud2(*msg, pcl_cloud2);

            // 保存 PointCloud2 数据
            savePointCloud2ToFile(pcl_cloud2, pointcloud2_file_);

            // 保存原始 PointCloud 数据
            savePointCloudToFile(*msg, pointcloud_file_);

            ROS_INFO("Saved PointCloud2 and PointCloud data.");

            message_count_++;
        }

        // 如果已经收到三条消息，关闭节点
        if (message_count_ >= 3)
        {
            ros::shutdown();
        }
    }

private:
    ros::Subscriber sub_;
    std::ofstream pointcloud_file_;
    std::ofstream pointcloud2_file_;
    std::atomic<int> message_count_;
    std::thread monitor_thread_;
    std::atomic<bool> shutdown_requested_;

    // 监控超时函数
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

    // 创建目录的辅助函数
    bool createDirectory(const std::string &path)
    {
        struct stat info;
        if (stat(path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR))
        {
            if (mkdir(path.c_str(), 0777) && errno != EEXIST)
            {
                return false;
            }
        }
        return true;
    }

    // 将 PointCloud 手动转换为 PointCloud2
    void convertPointCloudToPointCloud2(const sensor_msgs::PointCloud &input, sensor_msgs::PointCloud2 &output)
    {
        // 这里需要手动填充 PointCloud2 的字段
        pcl::PCLPointCloud2 pcl_cloud2;
        pcl_conversions::moveToPCL(input, pcl_cloud2);
        pcl_conversions::fromPCL(pcl_cloud2, output);
    }

    // 保存 PointCloud 数据到文件
    void savePointCloudToFile(const sensor_msgs::PointCloud &cloud, std::ofstream &file)
    {
        if (file.is_open())
        {
            file << "Message " << message_count_ + 1 << ":\n";
            file << "Header:\n";
            file << "  seq: " << cloud.header.seq << "\n";
            file << "  stamp: " << cloud.header.stamp << "\n";
            file << "  frame_id: " << cloud.header.frame_id << "\n";

            file << "Points:\n";
            for (const auto &point : cloud.points)
            {
                file << "  - x: " << point.x << ", y: " << point.y << ", z: " << point.z << "\n";
            }

            file << "Channels:\n";
            for (const auto &channel : cloud.channels)
            {
                file << "  - name: " << channel.name << "\n";
                file << "    values:\n";
                for (const auto &value : channel.values)
                {
                    file << "      - " << value << "\n";
                }
            }
            file << "\n";
        }
        else
        {
            ROS_ERROR("Failed to write to PointCloud file.");
        }
    }

    // 保存 PointCloud2 数据到文件
    void savePointCloud2ToFile(const sensor_msgs::PointCloud2 &msg, std::ofstream &file)
    {
        if (file.is_open())
        {
            file << "Message " << message_count_ + 1 << ":\n";
            file << "Header:\n";
            file << "  seq: " << msg.header.seq << "\n";
            file << "  stamp: " << msg.header.stamp << "\n";
            file << "  frame_id: " << msg.header.frame_id << "\n";

            file << "Height: " << msg.height << "\n";
            file << "Width: " << msg.width << "\n";

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

            file << "Data (hex):\n";
            for (size_t i = 0; i < msg.data.size(); ++i)
            {
                if (i % 16 == 0 && i != 0)
                    file << "\n";
                file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(msg.data[i]) << " ";
            }
            file << "\n";
        }
        else
        {
            ROS_ERROR("Failed to write to PointCloud2 file.");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_handler");
    PointCloudHandler handler;
    ros::spin();
    return 0;
}
