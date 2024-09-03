#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <atomic>

class PointCloudHandler
{
public:
    PointCloudHandler() : message_count_(0)
    {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("/ars548_process/point_cloud", 10, &PointCloudHandler::callback, this);

        // Create a directory named "txt"
        struct stat info;
        if (stat("txt", &info) != 0 || !(info.st_mode & S_IFDIR))
        {
            if (mkdir("txt", 0777) && errno != EEXIST)
            {
                ROS_ERROR("Unable to create 'txt' directory");
            }
        }

        // Open the output files for writing
        pointcloud_file_.open("txt/processor_pointcloud.txt");
        pointcloud2_file_.open("txt/processor_pointcloud2.txt");

        if (!pointcloud_file_.is_open() || !pointcloud2_file_.is_open())
        {
            ROS_ERROR("Unable to open output files for writing");
        }

        // Start a separate thread to monitor the timeout
        monitor_thread_ = std::thread(&PointCloudHandler::monitorTimeout, this);
    }

    ~PointCloudHandler()
    {
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
            // Save the raw PointCloud data to a txt file
            savePointCloudToFile(*msg, pointcloud_file_);

            // Convert PointCloud to PointCloud2
            sensor_msgs::PointCloud2 cloud2_msg;
            convertPointCloudToPointCloud2(msg, cloud2_msg);

            // Save the converted PointCloud2 data to a txt file
            savePointCloud2ToFile(cloud2_msg, pointcloud2_file_);

            ROS_INFO("Saved PointCloud data and converted PointCloud2 data");

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
    std::ofstream pointcloud_file_;
    std::ofstream pointcloud2_file_;
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

    // Convert sensor_msgs::PointCloud to sensor_msgs::PointCloud2
    void convertPointCloudToPointCloud2(const sensor_msgs::PointCloud::ConstPtr &cloud_msg, sensor_msgs::PointCloud2 &cloud2_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.header = pcl_conversions::toPCL(cloud_msg->header);

        // Convert points
        for (const auto &point : cloud_msg->points)
        {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            pcl_cloud.points.push_back(pcl_point);
        }

        // Convert pcl::PointCloud to pcl::PCLPointCloud2
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);

        // Calculate new point step and resize pcl_pc2.data accordingly
        int additional_step = 4 * cloud_msg->channels.size(); // 4 bytes for each float channel value
        pcl_pc2.point_step += additional_step;
        pcl_pc2.data.resize(pcl_pc2.width * pcl_pc2.height * pcl_pc2.point_step);

        // Add channels to pcl_pc2 as additional fields
        int offset = pcl_pc2.point_step - additional_step;
        for (const auto &channel : cloud_msg->channels)
        {
            pcl::PCLPointField field;
            field.name = channel.name;
            field.offset = offset;
            field.datatype = pcl::PCLPointField::FLOAT32;
            field.count = 1;

            pcl_pc2.fields.push_back(field);

            // Add channel data to pcl_pc2 data
            for (size_t i = 0; i < pcl_cloud.points.size(); ++i)
            {
                float value = channel.values[i];
                memcpy(&pcl_pc2.data[i * pcl_pc2.point_step + field.offset], &value, sizeof(float));
            }

            offset += 4; // FLOAT32 is 4 bytes
        }

        pcl_pc2.row_step = pcl_pc2.point_step * pcl_pc2.width;

        // Convert pcl::PCLPointCloud2 to sensor_msgs::PointCloud2
        pcl_conversions::fromPCL(pcl_pc2, cloud2_msg);
        cloud2_msg.header = cloud_msg->header;
    }

    // Save PointCloud data to a TXT file
    void savePointCloudToFile(const sensor_msgs::PointCloud &msg, std::ofstream &file)
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

            // Write points
            file << "Points:\n";
            for (const auto &point : msg.points)
            {
                file << "  - x: " << point.x << ", y: " << point.y << ", z: " << point.z << "\n";
            }

            // Write channels
            file << "Channels:\n";
            for (const auto &channel : msg.channels)
            {
                file << "  - name: " << channel.name << "\n";
                file << "    values:\n";
                for (const auto &value : channel.values)
                {
                    file << "      - " << value << "\n";
                }
            }

            file << "\n"; // Add a new line after each message
        }
        else
        {
            ROS_ERROR("Unable to write to PointCloud file");
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

            // Write data in hex format
            file << "Data (hex):\n";
            for (size_t i = 0; i < msg.data.size(); ++i)
            {
                if (i % 16 == 0 && i != 0)
                    file << "\n"; // New line every 16 bytes
                file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(msg.data[i]) << " ";
            }
            file << "\n"; // Add a new line after each message
        }
        else
        {
            ROS_ERROR("Unable to write to PointCloud2 file");
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
