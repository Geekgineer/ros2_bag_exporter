/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__GPS_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__GPS_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>

namespace rosbag2_exporter
{

class GPSHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  GPSHandler(const std::string & topic_dir, rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {}

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::NavSatFix gps_data;
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
    serializer.deserialize_message(&serialized_msg, &gps_data);

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << gps_data.header.stamp.sec << "-"
                << std::setw(9) << std::setfill('0') << gps_data.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Sanitize the topic name by removing the leading '/'
    std::string sanitized_topic = topic;
    if (!sanitized_topic.empty() && sanitized_topic[0] == '/') {
      sanitized_topic = sanitized_topic.substr(1);
    }

    // Create the full file path with '.csv' as the extension
    std::string filepath = topic_dir_ + "/" + timestamp + ".csv";

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = topic_dir_;
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
      std::filesystem::create_directories(dir_path);
    }

    // Open file and write GPS data as CSV
    std::ofstream outfile(filepath);
    if (!outfile.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write GPS data: %s", filepath.c_str());
      return;
    }

    // Write GPS data (latitude, longitude, altitude, position covariance)
    outfile << "timestamp," << "latitude,longitude,altitude,"
            << "covariance[0],covariance[1],covariance[2],covariance[3],covariance[4],covariance[5],covariance[6],covariance[7],covariance[8]" << std::endl;
    outfile << timestamp << ","
            << gps_data.latitude << "," << gps_data.longitude << "," << gps_data.altitude << ","
            << gps_data.position_covariance[0] << "," << gps_data.position_covariance[1] << "," << gps_data.position_covariance[2] << ","
            << gps_data.position_covariance[3] << "," << gps_data.position_covariance[4] << "," << gps_data.position_covariance[5] << ","
            << gps_data.position_covariance[6] << "," << gps_data.position_covariance[7] << "," << gps_data.position_covariance[8]
            << std::endl;

    outfile.close();

    RCLCPP_INFO(logger_, "Successfully wrote GPS data to %s", filepath.c_str());
  }

private:
  std::string topic_dir_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__GPS_HANDLER_HPP_
