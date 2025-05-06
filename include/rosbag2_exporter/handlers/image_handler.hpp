/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__IMAGE_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__IMAGE_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace rosbag2_exporter
{

class ImageHandler : public BaseHandler
{
public:
  // Constructor to accept logger and encoding, with a default value for encoding
  ImageHandler(const std::string & topic_dir,
               const std::string & encoding,
               rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
    // Validate or set default encoding if not provided
    if (encoding.empty()) {
      RCLCPP_WARN(logger, "No encoding provided. Defaulting to 'rgb8'.");
      encoding_ = "rgb8";  // Default to rgb8 if encoding is not provided
    } else {
      encoding_ = encoding;
    }
  }

  // Handle uncompressed image messages
  void process_message(const rclcpp::SerializedMessage & serialized_msg,
                     const std::string & topic,
                     size_t index) override
  {
      // Deserialize the incoming uncompressed image message
      sensor_msgs::msg::Image img;
      rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
      serializer.deserialize_message(&serialized_msg, &img);

      // Convert the sensor message to a cv::Mat image using cv_bridge
      cv_bridge::CvImagePtr cv_ptr;
      try {
          cv_ptr = cv_bridge::toCvCopy(img, encoding_);
      } catch (const cv_bridge::Exception & e) {
          RCLCPP_ERROR(logger_, "CV Bridge exception: %s. Using default encoding 'rgb8'.", e.what());

          // Attempt to fallback to the default 'rgb8' encoding
          try {
              cv_ptr = cv_bridge::toCvCopy(img, "rgb8");
              encoding_ = "rgb8";  // Update to fallback encoding
          } catch (const cv_bridge::Exception & e2) {
              RCLCPP_ERROR(logger_, "Fallback to 'rgb8' failed: %s", e2.what());
              return;
          }
      }

      // Apply color conversion based on encoding
      if (encoding_ == "rgb8") {
          // Convert from RGB to BGR for saving with OpenCV (OpenCV uses BGR)
          cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
      } else if (encoding_ == "bgr8") {
          // No conversion needed, OpenCV already uses BGR format
          RCLCPP_INFO(logger_, "Image is already in 'bgr8', no conversion applied.");
      } else if (encoding_ == "mono8" || encoding_ == "mono16") {
          // Grayscale images (mono8 or mono16), no color conversion needed
          RCLCPP_INFO(logger_, "Image is grayscale (%s), no color conversion applied.", encoding_.c_str());
      } else {
          RCLCPP_WARN(logger_, "Unsupported image encoding '%s'. Skipping color conversion.", encoding_.c_str());
      }

      // Save the image to file
      save_image(cv_ptr->image, topic, img.header.stamp);
  }

  // Handle compressed image messages
  void process_compressed_message(const rclcpp::SerializedMessage & serialized_msg,
                                  const std::string & topic,
                                  size_t index)
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

    // Create the full file path
    std::string filepath = topic_dir_ + "/" + timestamp + extension;

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = topic_dir_;
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
  std::string topic_dir_;
  std::string encoding_;

  // Helper function to save uncompressed images
  void save_image(const cv::Mat& image, const std::string& topic, const builtin_interfaces::msg::Time& timestamp)
  {
    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << timestamp.sec << "-"
                 << std::setw(9) << std::setfill('0') << timestamp.nanosec;
    std::string timestamp_str = ss_timestamp.str();

    // Create the full file path
    std::string filepath = topic_dir_ + "/" + timestamp_str + ".png";

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = topic_dir_;
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
      std::filesystem::create_directories(dir_path);
    }

    // Write the image to disk
    if (!cv::imwrite(filepath, image)) {
      RCLCPP_ERROR(logger_, "Failed to write image to %s", filepath.c_str());
    } else {
      RCLCPP_INFO(logger_, "Successfully wrote image to %s", filepath.c_str());
    }
  }
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__IMAGE_HANDLER_HPP_
