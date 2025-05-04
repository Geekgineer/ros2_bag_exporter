/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer (Template)
 * Adapted By: [Ziv Barcesat/Barcesat], [www.linkedin.com/in/ziv-barcesat]
 * Date: 01.05.2025
 * Description: Handler for exporting nav_msgs/msg/Odometry messages to CSV files.
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__ODOMETRY_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__ODOMETRY_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <nav_msgs/msg/odometry.hpp>           // Include the Odometry message type
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/logger.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <vector>

namespace rosbag2_exporter
{

class OdometryHandler : public BaseHandler
{
public:
  // Constructor to accept output directory and logger
  OdometryHandler(const std::string & output_dir, rclcpp::Logger logger)
  : BaseHandler(logger), output_dir_(output_dir)
  {}

  // Override the process_message function for Odometry messages
  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                       const std::string & topic,
                       size_t index) override
  {
    // 1. Deserialize the incoming message
    nav_msgs::msg::Odometry odom_data;
    rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
    serializer.deserialize_message(&serialized_msg, &odom_data);

    // 2. Create a timestamp string from the message header
    std::stringstream ss_timestamp;
    ss_timestamp << odom_data.header.stamp.sec << "-"
                 << std::setw(9) << std::setfill('0') << odom_data.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // 3. Sanitize the topic name (remove leading '/')
    std::string sanitized_topic = topic;
    if (!sanitized_topic.empty() && sanitized_topic[0] == '/') {
      sanitized_topic = sanitized_topic.substr(1);
    }

    // 4. Construct the directory path based on the topic
    std::filesystem::path dir_path = output_dir_ + "/" + sanitized_topic;

    // 5. Ensure the directory exists, create if necessary
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
      try {
        std::filesystem::create_directories(dir_path);
      } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(logger_, "Failed to create directory %s: %s", dir_path.c_str(), e.what());
        return; // Stop processing if directory creation fails
      }
    }

    // 6. Construct the full file path with '.csv' extension
    std::string filepath = dir_path.string() + "/" + timestamp + ".csv";

    // 7. Open the output file stream
    std::ofstream outfile(filepath);
    if (!outfile.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write Odometry data: %s", filepath.c_str());
      return;
    }

    // 8. Write the CSV header
    outfile << "msg_timestamp_sec,msg_timestamp_nanosec,msg_frame_id,"
            << "pos_x,pos_y,pos_z,"
            << "orient_x,orient_y,orient_z,orient_w,"
            << "linear_vel_x,linear_vel_y,linear_vel_z,"
            << "angular_vel_x,angular_vel_y,angular_vel_z" << std::endl;

    // 9. Write the Odometry data to the CSV file
    outfile << odom_data.header.stamp.sec << "," << odom_data.header.stamp.nanosec << ","
            << odom_data.header.frame_id << ","
            << odom_data.pose.pose.position.x << "," << odom_data.pose.pose.position.y << "," << odom_data.pose.pose.position.z << ","
            << odom_data.pose.pose.orientation.x << "," << odom_data.pose.pose.orientation.y << "," << odom_data.pose.pose.orientation.z << "," << odom_data.pose.pose.orientation.w << ","
            << odom_data.twist.twist.linear.x << "," << odom_data.twist.twist.linear.y << "," << odom_data.twist.twist.linear.z << ","
            << odom_data.twist.twist.angular.x << "," << odom_data.twist.twist.angular.y << "," << odom_data.twist.twist.angular.z
            << std::endl;

    // 10. Close the file
    outfile.close();

    // 11. Log success
    RCLCPP_INFO(logger_, "Successfully wrote Odometry data (message #%zu) to %s", index, filepath.c_str());
  }

private:
  std::string output_dir_; // Directory where files will be saved
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__ODOMETRY_HANDLER_HPP_