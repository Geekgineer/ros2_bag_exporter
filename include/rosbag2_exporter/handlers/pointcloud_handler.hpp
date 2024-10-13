/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__POINTCLOUD_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__POINTCLOUD_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <pcl/point_types.h>

namespace rosbag2_exporter
{

class PointCloudHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  PointCloudHandler(const std::string & output_dir, rclcpp::Logger logger)
  : BaseHandler(logger), output_dir_(output_dir)
  {}

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    sensor_msgs::msg::PointCloud2 pc2;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    serializer.deserialize_message(&serialized_msg, &pc2);

    // Check if the point cloud has an intensity field
    bool has_intensity = false;
    for (const auto& field : pc2.fields) {
      if (field.name == "intensity") {
        has_intensity = true;
        break;
      }
    }

    // Create the point cloud and convert based on whether intensity is present
    if (has_intensity) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(pc2, *cloud);
      save_pointcloud_to_file<pcl::PointXYZI>(cloud, topic, pc2, index);  // Explicitly specify template type
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(pc2, *cloud);
      save_pointcloud_to_file<pcl::PointXYZ>(cloud, topic, pc2, index);  // Explicitly specify template type
    }
  }

private:
  std::string output_dir_;

  // Templated function to save point cloud to file
  template<typename PointT>
  void save_pointcloud_to_file(typename pcl::PointCloud<PointT>::Ptr cloud,
                               const std::string & topic,
                               const sensor_msgs::msg::PointCloud2 & pc2,
                               size_t index)
  {
    // Construct timestamp string
    std::stringstream ss_timestamp;
    ss_timestamp << pc2.header.stamp.sec << "-"
                 << std::setw(9) << std::setfill('0') << pc2.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Log the processing
    RCLCPP_INFO(logger_, "Processing PointCloud2 message at timestamp: %s #%zu", timestamp.c_str(), index);

    // Ensure the directory exists
    std::string topic_dir = output_dir_ + "/" + topic.substr(1);
    std::filesystem::create_directories(topic_dir);

    // Construct filename
    std::string filename = topic_dir + "/" + timestamp + ".pcd";

    // Save the point cloud
    if (pcl::io::savePCDFileBinary(filename, *cloud) == -1) {
      RCLCPP_ERROR(logger_, "Failed to write PCD file to %s", filename.c_str());
    }
  }
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__POINTCLOUD_HANDLER_HPP_
