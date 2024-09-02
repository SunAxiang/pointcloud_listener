import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
import threading
import os


class PointCloudListener:
    def __init__(self):
        # Ensure the 'txt' directory exists
        os.makedirs('txt', exist_ok=True)

        self.count_pcl = 0
        self.max_count_pcl = 2
        self.topic_pcl = "/radar_enhanced_pcl"
        self.file_pcl = open(
            f"txt/{self.topic_pcl.replace('/', '_')}.txt", "w")

        self.count_pcl2 = 0
        self.max_count_pcl2 = 2
        self.topic_pcl2 = "/ars548"
        self.file_pcl2 = open(
            f"txt/{self.topic_pcl2.replace('/', '_')}.txt", "w")

        self.sub_pcl = rospy.Subscriber(
            self.topic_pcl, PointCloud, self.callback_pcl)
        self.sub_pcl2 = rospy.Subscriber(
            self.topic_pcl2, PointCloud2, self.callback_pcl2)

        self.timer = threading.Timer(30.0, self.shutdown_due_to_timeout)
        self.timer.start()

    def callback_pcl(self, msg):
        if self.count_pcl < self.max_count_pcl:
            rospy.loginfo(f"Received PointCloud message {self.count_pcl + 1}")
            self.file_pcl.write(f"Message {self.count_pcl + 1}:\n")
            self.file_pcl.write(
                f"Header:\n  seq: {msg.header.seq}\n  stamp: {msg.header.stamp}\n  frame_id: {msg.header.frame_id}\n")

            self.file_pcl.write("Points:\n")
            for point in msg.points:
                self.file_pcl.write(
                    f"  - x: {point.x}, y: {point.y}, z: {point.z}\n")

            self.file_pcl.write("Channels:\n")
            for channel in msg.channels:
                self.file_pcl.write(f"  - name: {channel.name}\n")
                self.file_pcl.write("    values:\n")
                for value in channel.values:
                    self.file_pcl.write(f"      - {value}\n")

            self.file_pcl.write("\n")
            self.count_pcl += 1

            if self.count_pcl == self.max_count_pcl:
                self.file_pcl.close()
                rospy.loginfo(
                    f"PointCloud messages collected. Output saved to '{self.file_pcl.name}'")
                self.check_shutdown()

    def callback_pcl2(self, msg):
        if self.count_pcl2 < self.max_count_pcl2:
            rospy.loginfo(
                f"Received PointCloud2 message {self.count_pcl2 + 1}")
            self.file_pcl2.write(f"Message {self.count_pcl2 + 1}:\n")
            self.file_pcl2.write(
                f"Header:\n  seq: {msg.header.seq}\n  stamp: {msg.header.stamp}\n  frame_id: {msg.header.frame_id}\n")
            self.file_pcl2.write(f"Height: {msg.height}\n")
            self.file_pcl2.write(f"Width: {msg.width}\n")

            self.file_pcl2.write("Fields:\n")
            for field in msg.fields:
                self.file_pcl2.write(f"  - name: {field.name}\n")
                self.file_pcl2.write(f"    offset: {field.offset}\n")
                self.file_pcl2.write(f"    datatype: {field.datatype}\n")
                self.file_pcl2.write(f"    count: {field.count}\n")

            self.file_pcl2.write(f"Is Big Endian: {msg.is_bigendian}\n")
            self.file_pcl2.write(f"Point Step: {msg.point_step}\n")
            self.file_pcl2.write(f"Row Step: {msg.row_step}\n")

            data_list = list(msg.data)
            self.file_pcl2.write(f"Data:\n  {data_list}\n")

            self.file_pcl2.write(f"Is Dense: {msg.is_dense}\n")

            self.file_pcl2.write("\n")
            self.count_pcl2 += 1

            if self.count_pcl2 == self.max_count_pcl2:
                self.file_pcl2.close()
                rospy.loginfo(
                    f"PointCloud2 messages collected. Output saved to '{self.file_pcl2.name}'")
                self.check_shutdown()

    def check_shutdown(self):
        if self.count_pcl >= self.max_count_pcl or self.count_pcl2 >= self.max_count_pcl2:
            rospy.loginfo("Collected all messages. Shutting down the node.")
            self.timer.cancel()  # Cancel the timer if we finished in time
            rospy.signal_shutdown(
                "Collected all messages. Shutting down the node.")

    def shutdown_due_to_timeout(self):
        rospy.loginfo("Timeout reached. Shutting down the node.")
        rospy.signal_shutdown(
            "Timeout reached. Shutting down the node due to timeout.")

    def shutdown(self):
        rospy.loginfo("Shutting down the node.")
        if self.file_pcl and not self.file_pcl.closed:
            self.file_pcl.close()
        if self.file_pcl2 and not self.file_pcl2.closed:
            self.file_pcl2.close()


if __name__ == '__main__':
    rospy.init_node('pointcloud_listener', anonymous=True)
    listener = PointCloudListener()

    rospy.on_shutdown(listener.shutdown)

    rospy.spin()
