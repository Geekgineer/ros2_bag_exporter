/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>

namespace rosbag2_exporter
{

class IMUHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  IMUHandler(const std::string & output_dir, rclcpp::Logger logger)
  : BaseHandler(logger), output_dir_(output_dir)
  {}

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::Imu imu_data;
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    serializer.deserialize_message(&serialized_msg, &imu_data);

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << imu_data.header.stamp.sec << "-"
                << std::setw(9) << std::setfill('0') << imu_data.header.stamp.nanosec;
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

    // Open file and write IMU data as CSV
    std::ofstream outfile(filepath);
    if (!outfile.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write IMU data: %s", filepath.c_str());
      return;
    }

    // Write IMU data (orientation, angular velocity, linear acceleration)
    outfile << "timestamp," << "orientation_x,orientation_y,orientation_z,orientation_w,"
            << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
            << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z" << std::endl;
    outfile << timestamp << ","
            << imu_data.orientation.x << "," << imu_data.orientation.y << "," << imu_data.orientation.z << "," << imu_data.orientation.w << ","
            << imu_data.angular_velocity.x << "," << imu_data.angular_velocity.y << "," << imu_data.angular_velocity.z << ","
            << imu_data.linear_acceleration.x << "," << imu_data.linear_acceleration.y << "," << imu_data.linear_acceleration.z
            << std::endl;

    outfile.close();

    RCLCPP_INFO(logger_, "Successfully wrote IMU data to %s", filepath.c_str());
  }

private:
  std::string output_dir_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_
