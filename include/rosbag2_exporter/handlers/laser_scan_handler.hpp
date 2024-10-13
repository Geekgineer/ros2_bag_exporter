/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__LASER_SCAN_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__LASER_SCAN_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>

namespace rosbag2_exporter
{

class LaserScanHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  LaserScanHandler(const std::string & output_dir, rclcpp::Logger logger)
  : BaseHandler(logger), output_dir_(output_dir)
  {}

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::LaserScan laser_scan;
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;
    serializer.deserialize_message(&serialized_msg, &laser_scan);

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << laser_scan.header.stamp.sec << "-"
                << std::setw(9) << std::setfill('0') << laser_scan.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Sanitize the topic name by removing the leading '/'
    std::string sanitized_topic = topic;
    if (!sanitized_topic.empty() && sanitized_topic[0] == '/') {
      sanitized_topic = sanitized_topic.substr(1);
    }

    // Create the full file path with '.csv' as the extension
    std::string filepath = output_dir_ + "/" + sanitized_topic + "/" + timestamp + ".csv";

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = output_dir_ + "/" + sanitized_topic;
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
      std::filesystem::create_directories(dir_path);
    }

    // Open file and write LaserScan data as CSV
    std::ofstream outfile(filepath);
    if (!outfile.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write LaserScan data: %s", filepath.c_str());
      return;
    }

    // Write LaserScan data headers
    outfile << "timestamp,angle_min,angle_max,angle_increment,time_increment,scan_time,range_min,range_max\n";
    outfile << timestamp << ","
            << laser_scan.angle_min << "," << laser_scan.angle_max << "," << laser_scan.angle_increment << ","
            << laser_scan.time_increment << "," << laser_scan.scan_time << "," << laser_scan.range_min << ","
            << laser_scan.range_max << std::endl;

    // Write ranges and intensities if available
    outfile << "ranges,";
    for (size_t i = 0; i < laser_scan.ranges.size(); ++i) {
      outfile << laser_scan.ranges[i];
      if (i != laser_scan.ranges.size() - 1) {
        outfile << ",";
      }
    }
    outfile << "\n";

    if (!laser_scan.intensities.empty()) {
      outfile << "intensities,";
      for (size_t i = 0; i < laser_scan.intensities.size(); ++i) {
        outfile << laser_scan.intensities[i];
        if (i != laser_scan.intensities.size() - 1) {
          outfile << ",";
        }
      }
      outfile << "\n";
    }

    outfile.close();
    RCLCPP_INFO(logger_, "Successfully wrote LaserScan data to %s", filepath.c_str());
  }

private:
  std::string output_dir_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__LASER_SCAN_HANDLER_HPP_
