/*
 * This file is based on the original work by:
 * 
 * Original Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Original Date: 13.10.2024
 * 
 * Adapted and Modified By: Hector Cruz, hcruzgo@ed.ac.uk
 * Adaptation Date: 24.01.2025
 * 
 * Description of Adaptations:
 * - Used as a template to support CompressedImages handling
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__COMPRESSED_IMAGE_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__COMPRESSED_IMAGE_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace rosbag2_exporter
{

class CompressedImageHandler : public BaseHandler
{
public:

  CompressedImageHandler(const std::string & output_dir,
               const std::string & encoding,
               rclcpp::Logger logger)
  : BaseHandler(logger), output_dir_(output_dir)
  {}

  // Handle compressed image messages
  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                                  const std::string & topic,
                                  size_t index) override
  {
    // Deserialize the incoming compressed image message
    sensor_msgs::msg::CompressedImage compressed_img;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
    serializer.deserialize_message(&serialized_msg, &compressed_img);

    // Determine file extension based on the compressed image format
    std::string extension;
    if (compressed_img.format.find("jpeg") != std::string::npos || compressed_img.format.find("jpg") != std::string::npos) {
      extension = ".jpg";
    } else if (compressed_img.format.find("png") != std::string::npos) {
      extension = ".png";
    } else {
      RCLCPP_WARN(logger_, "Unknown compressed image format: %s. Defaulting to '.jpg'", compressed_img.format.c_str());
      extension = ".jpg";  // Default to JPEG if unknown
    }

    // Create a timestamped filename and save compressed image directly
    std::stringstream ss_timestamp;
    ss_timestamp << compressed_img.header.stamp.sec << "-"
                 << std::setw(9) << std::setfill('0') << compressed_img.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    std::string sanitized_topic = topic;
    // RCLCPP_WARN(logger_, "Topic-> %s", sanitized_topic.c_str());
    if (!sanitized_topic.empty() && sanitized_topic[0] == '/') {
      sanitized_topic = sanitized_topic.substr(1);
    }

    // Create the full file path
    std::string filepath = output_dir_ + "/" + sanitized_topic + "/" + timestamp + extension;

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = output_dir_ + "/" + sanitized_topic;
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
      std::filesystem::create_directories(dir_path);
    }

    // Save the compressed image data directly to file
    std::ofstream outfile(filepath, std::ios::binary);
    if (!outfile.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write compressed image: %s", filepath.c_str());
      return;
    }
    outfile.write(reinterpret_cast<const char*>(compressed_img.data.data()), compressed_img.data.size());
    outfile.close();

    RCLCPP_INFO(logger_, "Successfully wrote compressed image to %s", filepath.c_str());
  }


private:
  std::string output_dir_;
  // Defined to comply with class parent but not needed
  std::string encoding_; 

};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__COMPRESSED_IMAGE_HANDLER_HPP_
