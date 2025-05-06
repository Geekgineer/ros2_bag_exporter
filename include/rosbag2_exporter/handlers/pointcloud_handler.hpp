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
  // Constructor to accept logger and save_mode
  PointCloudHandler(const std::string & topic_dir, const std::string & save_mode, rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir), save_mode_(save_mode)
  {}

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    sensor_msgs::msg::PointCloud2 pc2;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    serializer.deserialize_message(&serialized_msg, &pc2);

    // Check for fields
    bool has_intensity = false;
    bool has_rgb = false;
    bool has_rgba = false;
    for (const auto& field : pc2.fields) {
      if (field.name == "intensity") has_intensity = true;
      if (field.name == "rgb") has_rgb = true;
      if (field.name == "rgba") has_rgba = true;
    }

    std::string mode = save_mode_;
    if (mode == "auto") {
      if (has_intensity) mode = "intensity";
      else if (has_rgb) mode = "rgb";
      else if (has_rgba) mode = "rgba";
      else mode = "xyz";
    }

    if (mode == "intensity" && has_intensity) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(pc2, *cloud);
      save_pointcloud_to_file<pcl::PointXYZI>(cloud, topic, pc2, index);
    } else if (mode == "rgb" && has_rgb) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(pc2, *cloud);
      save_pointcloud_to_file<pcl::PointXYZRGB>(cloud, topic, pc2, index);
    } else if (mode == "rgba" && has_rgba) {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::fromROSMsg(pc2, *cloud);
      save_pointcloud_to_file<pcl::PointXYZRGBA>(cloud, topic, pc2, index);
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(pc2, *cloud);
      save_pointcloud_to_file<pcl::PointXYZ>(cloud, topic, pc2, index);
    }
  }

private:
  std::string topic_dir_;
  std::string save_mode_;

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

    // Create the full file path
    std::string filepath = topic_dir_ + "/" + timestamp + ".pcd";

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = topic_dir_;
    if (!std::filesystem::exists(dir_path)) {
        RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
        std::filesystem::create_directories(dir_path);
    }

    // Save the point cloud
    if (pcl::io::savePCDFileBinary(filepath, *cloud) == -1) {
      RCLCPP_ERROR(logger_, "Failed to write PCD file to %s", filepath.c_str());
    }
  }
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__POINTCLOUD_HANDLER_HPP_
