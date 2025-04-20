/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer (Template)
 * Adapted By: [Ziv Barcesat/Barcesat], [www.linkedin.com/in/ziv-barcesat]
 * Date: [Current Date, e.g., 16.05.2024]
 * Description: Handler for exporting nav_msgs/msg/Path messages to CSV files.
 */

 #ifndef ROSBAG2_EXPORTER__HANDLERS__PATH_HANDLER_HPP_
 #define ROSBAG2_EXPORTER__HANDLERS__PATH_HANDLER_HPP_

 #include "rosbag2_exporter/handlers/base_handler.hpp"
 #include <nav_msgs/msg/path.hpp>              // Include the Path message type
 #include <geometry_msgs/msg/pose_stamped.hpp> // Included by nav_msgs/msg/path.hpp, but good practice
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

 class PathHandler : public BaseHandler
 {
 public:
   // Constructor to accept output directory and logger
   PathHandler(const std::string & output_dir, rclcpp::Logger logger)
   : BaseHandler(logger), output_dir_(output_dir)
   {}

   // Override the process_message function for Path messages
   void process_message(const rclcpp::SerializedMessage & serialized_msg,
                       const std::string & topic,
                       size_t index) override
   {
     // 1. Deserialize the incoming message
     nav_msgs::msg::Path path_data;
     rclcpp::Serialization<nav_msgs::msg::Path> serializer;
     serializer.deserialize_message(&serialized_msg, &path_data);

     // 2. Create a timestamp string from the message header
     std::stringstream ss_timestamp;
     ss_timestamp << path_data.header.stamp.sec << "-"
                  << std::setw(9) << std::setfill('0') << path_data.header.stamp.nanosec;
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
       RCLCPP_ERROR(logger_, "Failed to open file to write Path data: %s", filepath.c_str());
       return;
     }

     // 8. Write the CSV header
     // Include message header info and details for each pose in the path
     outfile << "msg_timestamp_sec,msg_timestamp_nanosec,msg_frame_id,"
             << "pose_timestamp_sec,pose_timestamp_nanosec,pose_frame_id," // PoseStamped also has header
             << "pos_x,pos_y,pos_z,"
             << "orient_x,orient_y,orient_z,orient_w" << std::endl;

     // 9. Iterate through each PoseStamped in the path's poses vector and write to CSV
     for (const auto& pose_stamped : path_data.poses) {
       outfile << path_data.header.stamp.sec << "," << path_data.header.stamp.nanosec << ","
               << path_data.header.frame_id << "," // Path message's frame_id
               << pose_stamped.header.stamp.sec << "," << pose_stamped.header.stamp.nanosec << ","
               << pose_stamped.header.frame_id << "," // PoseStamped's frame_id
               << pose_stamped.pose.position.x << "," << pose_stamped.pose.position.y << "," << pose_stamped.pose.position.z << ","
               << pose_stamped.pose.orientation.x << "," << pose_stamped.pose.orientation.y << "," << pose_stamped.pose.orientation.z << "," << pose_stamped.pose.orientation.w
               << std::endl;
     }

     // 10. Close the file
     outfile.close();

     // 11. Log success
     RCLCPP_INFO(logger_, "Successfully wrote Path data (message #%zu, %zu poses) to %s",
                 index, path_data.poses.size(), filepath.c_str());
   }

 private:
   std::string output_dir_; // Directory where files will be saved
 };

 }  // namespace rosbag2_exporter

 #endif  // ROSBAG2_EXPORTER__HANDLERS__PATH_HANDLER_HPP_
