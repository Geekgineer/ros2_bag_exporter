/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__IR_IMAGE_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__IR_IMAGE_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace rosbag2_exporter
{

class IRImageHandler : public BaseHandler
{
public:
  // Constructor to accept logger and encoding with a default fallback for encoding
  IRImageHandler(const std::string & output_dir,
                const std::string & encoding,
                rclcpp::Logger logger)
  : BaseHandler(logger), output_dir_(output_dir)
  {
    // Validate or set default encoding if not provided
    if (encoding.empty()) {
      RCLCPP_WARN(logger, "No encoding provided. Defaulting to 'mono16'.");
      encoding_ = "mono16";  // Default encoding for IR images
    } else {
      encoding_ = encoding;
    }
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                      const std::string & topic,
                      size_t index) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::Image img;
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    serializer.deserialize_message(&serialized_msg, &img);

    // Convert the sensor message to a cv::Mat image using cv_bridge
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, encoding_);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(logger_, "CV Bridge exception: %s. Using default encoding 'mono16'.", e.what());

      // Attempt to fallback to the default 'mono16' encoding
      try {
        cv_ptr = cv_bridge::toCvCopy(img, "mono16");
        encoding_ = "mono16";  // Update to fallback encoding
      } catch (const cv_bridge::Exception & e2) {
        RCLCPP_ERROR(logger_, "Fallback to 'mono16' failed: %s", e2.what());
        return;
      }
    }

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << img.header.stamp.sec << "-"
                << std::setw(9) << std::setfill('0') << img.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Sanitize the topic name by removing the leading '/'
    std::string sanitized_topic = topic;
    if (!sanitized_topic.empty() && sanitized_topic[0] == '/') {
      sanitized_topic = sanitized_topic.substr(1);
    }

    // Create the full file path with '.png' as the extension
    std::string filepath = output_dir_ + "/" + sanitized_topic + "/" + timestamp + ".png";

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = output_dir_ + "/" + sanitized_topic;
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
      std::filesystem::create_directories(dir_path);
    }

    // Write the image to disk
    if (!cv::imwrite(filepath, cv_ptr->image)) {
      RCLCPP_ERROR(logger_, "Failed to write IR image to %s", filepath.c_str());
    } else {
      RCLCPP_INFO(logger_, "Successfully wrote IR image to %s", filepath.c_str());
    }
  }

private:
  std::string output_dir_;
  std::string encoding_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__IR_IMAGE_HANDLER_HPP_
