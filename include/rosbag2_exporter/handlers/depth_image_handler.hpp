/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__DEPTH_IMAGE_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__DEPTH_IMAGE_HANDLER_HPP_

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace rosbag2_exporter
{

class DepthImageHandler : public BaseHandler
{
public:
  // Constructor to accept logger and encoding with default fallback
  DepthImageHandler(const std::string & topic_dir,
                   const std::string & encoding,
                   rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
    // Validate or set default encoding if not provided
    if (encoding.empty()) {
      RCLCPP_WARN(logger, "No encoding provided. Defaulting to '16UC1'.");
      encoding_ = "16UC1";  // Default encoding for depth images
    } else {
      encoding_ = encoding;
    }
  }

  // Helper function to check if encoding is color (bgr8/rgb8)
  bool isColorEncoding(const std::string & enc) {
    return (enc == "bgr8" || enc == "rgb8");
  }

  // Helper function to check if encoding is grayscale (mono8/8UC1)
  bool isGrayscaleEncoding(const std::string & enc) {
    return (enc == "mono8" || enc == "8UC1");
  }

  // Convert depth image to visualization format
  cv::Mat convertDepthToVisualization(const cv::Mat& depth_img) {
    cv::Mat visualization;
    double min_val, max_val;
    cv::minMaxLoc(depth_img, &min_val, &max_val);

    // Skip zero values in min/max computation for better normalization
    cv::Mat mask = depth_img > 0;
    if (cv::countNonZero(mask) > 0) {
      cv::minMaxLoc(depth_img, &min_val, &max_val, nullptr, nullptr, mask);
    }

    // Normalize to 0-255 range, keeping zero values as zero
    cv::Mat normalized;
    depth_img.convertTo(normalized, CV_8UC1, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

    if (isColorEncoding(encoding_)) {
      // Apply colormap for color visualization
      cv::applyColorMap(normalized, visualization, cv::COLORMAP_JET);

      // Keep zero values as black
      if (cv::countNonZero(mask) < depth_img.total()) {
        visualization.setTo(cv::Vec3b(0,0,0), ~mask);
      }

      // Convert to RGB if needed
      if (encoding_ == "rgb8") {
        cv::cvtColor(visualization, visualization, cv::COLOR_BGR2RGB);
      }
    } else if (isGrayscaleEncoding(encoding_)) {
      visualization = normalized;
    }

    return visualization;
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
      // Always read as 16UC1 first to get raw depth data
      cv_ptr = cv_bridge::toCvCopy(img, "16UC1");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(logger_, "CV Bridge exception: %s", e.what());
      return;
    }

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << img.header.stamp.sec << "-"
                << std::setw(9) << std::setfill('0') << img.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Ensure the directory exists, create if necessary
    std::filesystem::path dir_path = topic_dir_;
    if (!std::filesystem::exists(dir_path)) {
      RCLCPP_INFO(logger_, "Creating directory: %s", dir_path.c_str());
      std::filesystem::create_directories(dir_path);
    }

    // Process and save the image based on encoding
    cv::Mat output_image;
    if (isColorEncoding(encoding_) || isGrayscaleEncoding(encoding_)) {
      output_image = convertDepthToVisualization(cv_ptr->image);
    } else {
      // For raw depth, keep original
      output_image = cv_ptr->image;
    }

    // Create the full file path
    std::string filepath = topic_dir_ + "/" + timestamp + ".png";

    // Save the image
    if (cv::imwrite(filepath, output_image)) {
      RCLCPP_INFO(logger_, "Processing depth image #%zu: SAVED to %s",
                  index, filepath.c_str());
    } else {
      RCLCPP_ERROR(logger_, "Processing depth image #%zu: FAILED to save to %s",
                   index, filepath.c_str());
    }
  }

private:
  std::string topic_dir_;
  std::string encoding_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__DEPTH_IMAGE_HANDLER_HPP_
