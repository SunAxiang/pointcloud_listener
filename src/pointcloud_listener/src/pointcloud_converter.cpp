#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl_conversions/pcl_conversions.h>
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
        sub_ = nh.subscribe("/point_cloud_raw", 10, &PointCloudConverter::callback, this);

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

            // 将 PointCloud2 转换为 PointCloud
            sensor_msgs::PointCloud cloud;
            convertPointCloud2ToPointCloud(*msg, cloud);

            // 保存转换后的 PointCloud 数据到文件
            savePointCloudToFile(cloud, converted_file_);

            ROS_INFO("Saved PointCloud2 data and converted PointCloud data");

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

    // 保存转换后的 PointCloud 数据到文件
    void savePointCloudToFile(const sensor_msgs::PointCloud &cloud, std::ofstream &file)
    {
        if (file.is_open())
        {
            file << "Message " << message_count_ + 1 << ":\n";
            file << "Header:\n";
            file << "  seq: " << cloud.header.seq << "\n";
            file << "  stamp: " << cloud.header.stamp << "\n";
            file << "  frame_id: " << cloud.header.frame_id << "\n";

            // 写入点信息
            file << "Points:\n";
            for (const auto &point : cloud.points)
            {
                file << "  - x: " << point.x << ", y: " << point.y << ", z: " << point.z << "\n";
            }

            // 写入通道信息
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
            ROS_ERROR("Unable to write to converted PointCloud file");
        }
    }

    // 将 PointCloud2 转换为 PointCloud，并解析非 xyz 字段到 channels 中
    void convertPointCloud2ToPointCloud(const sensor_msgs::PointCloud2 &input, sensor_msgs::PointCloud &output)
    {
        output.header = input.header;

        // 保留点和通道的空间
        output.points.reserve(input.width * input.height);
        std::vector<sensor_msgs::ChannelFloat32> channels(input.fields.size());

        for (size_t i = 0; i < input.fields.size(); ++i)
        {
            const auto &field = input.fields[i];
            if (field.name != "x" && field.name != "y" && field.name != "z")
            {
                channels[i].name = field.name;
                channels[i].values.reserve(input.width * input.height);
            }
        }

        for (size_t i = 0; i < input.width * input.height; ++i)
        {
            geometry_msgs::Point32 point;

            // 提取 xyz 数据
            memcpy(&point.x, &input.data[i * input.point_step + input.fields[0].offset], sizeof(float));
            memcpy(&point.y, &input.data[i * input.point_step + input.fields[1].offset], sizeof(float));
            memcpy(&point.z, &input.data[i * input.point_step + input.fields[2].offset], sizeof(float));

            output.points.push_back(point);

            // 提取非 xyz 数据
            for (size_t j = 0; j < input.fields.size(); ++j)
            {
                const auto &field = input.fields[j];
                if (field.name != "x" && field.name != "y" && field.name != "z")
                {
                    float value;
                    memcpy(&value, &input.data[i * input.point_step + field.offset], sizeof(float));
                    channels[j].values.push_back(value);
                }
            }
        }

        // 将通道添加到输出点云
        for (const auto &channel : channels)
        {
            if (!channel.name.empty())
            {
                output.channels.push_back(channel);
            }
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
