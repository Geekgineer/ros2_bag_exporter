/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rosbag2_exporter/bag_exporter.hpp"

namespace rosbag2_exporter
{

BagExporter::BagExporter(const rclcpp::NodeOptions & options)
: Node("rosbag2_exporter", options)
{

  // Find the package share directory
  std::string package_share_directory;
  try {
      package_share_directory = ament_index_cpp::get_package_share_directory("ros2_bag_exporter");
  } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Package share directory not found: %s", e.what());
      rclcpp::shutdown();
      return;
  }
    
  // Declare and get the config_file parameter (absolute path)
  std::string config_file = this->declare_parameter<std::string>(
      "config_file", package_share_directory + "/config/exporter_config.yaml");

  // Load configuration
  load_configuration(config_file);

  // Setup handlers based on topics
  setup_handlers();

  // Start exporting
  export_bag();
}

void BagExporter::load_configuration(const std::string & config_file)
{
  try {
    RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

    YAML::Node config = YAML::LoadFile(config_file);
    bag_path_ = config["bag_path"].as<std::string>();
    output_dir_ = config["output_dir"].as<std::string>();
    storage_id_ = config["storage_id"].as<std::string>();

    for (const auto & topic : config["topics"]) {
      TopicConfig tc;
      tc.name = topic["name"].as<std::string>();
      std::string type = topic["type"].as<std::string>();

      tc.sample_interval = topic["sample_interval"] ? topic["sample_interval"].as<int>() : 1;  // Default to 1 (write every message)

      if (type == "PointCloud2") {
        tc.type = MessageType::PointCloud2;
      } else if (type == "Image") {
        tc.type = MessageType::Image;
        tc.encoding = topic["encoding"] ? topic["encoding"].as<std::string>() : "rgb8"; // default encoding
      } else if (type == "CompressedImage") {
        tc.type = MessageType::CompressedImage;
        tc.encoding = topic["encoding"] ? topic["encoding"].as<std::string>() : "rgb8"; // default encoding
      } else if (type == "DepthImage") {
        tc.type = MessageType::DepthImage;
        tc.encoding = topic["encoding"] ? topic["encoding"].as<std::string>() : "16UC1"; // default encoding
      } else if (type == "IMU") {
        tc.type = MessageType::IMU;
      } else if (type == "GPS") {
        tc.type = MessageType::GPS;
      } else if (type == "LaserScan") {
        tc.type = MessageType::LaserScan;
      } else {
        tc.type = MessageType::Unknown;
      }

      topics_.push_back(tc);
    }
  } catch (const YAML::BadFile & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load configuration: %s", e.what());
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "An error occurred while loading configuration: %s", e.what());
    rclcpp::shutdown();
  }
}

void BagExporter::setup_handlers()
{
  for (const auto & topic : topics_) {
    // Ensure topic name starts with '/'
    std::string sanitized_topic = topic.name;
    if (sanitized_topic.empty() || sanitized_topic[0] != '/') {
      sanitized_topic = "/" + sanitized_topic;
    }

    // Create directory for each topic
    std::string topic_dir = output_dir_ + "/" + sanitized_topic.substr(1); // Remove leading '/'
    std::filesystem::create_directories(topic_dir);

    // Initialize handler based on message type
    if (topic.type == MessageType::PointCloud2) {
      auto handler = std::make_shared<PointCloudHandler>(topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::Image) {
      auto handler = std::make_shared<ImageHandler>(topic_dir, topic.encoding, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::CompressedImage) {
      auto handler = std::make_shared<CompressedImageHandler>(topic_dir, topic.encoding, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::DepthImage) {
      auto handler = std::make_shared<DepthImageHandler>(topic_dir, topic.encoding, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::IRImage) {
      auto handler = std::make_shared<IRImageHandler>(topic_dir, topic.encoding, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::IMU) {
      auto handler = std::make_shared<IMUHandler>(topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::GPS) {
      auto handler = std::make_shared<GPSHandler>(topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::LaserScan) {
      auto handler = std::make_shared<LaserScanHandler>(topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else {
      RCLCPP_WARN(this->get_logger(), "Unsupported message type for topic '%s'. Skipping.", topic.name.c_str());
    }
  }
}
void BagExporter::export_bag()
{
  // Initialize reader
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path_;
  storage_options.storage_id = storage_id_;
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open bag: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  // Get topic metadata
  auto metadata = reader.get_all_topics_and_types();

  // Initialize handlers based on available topics
  for (auto & [topic_name, handler] : handlers_) {
    auto it = std::find_if(metadata.begin(), metadata.end(),
      [&topic_name](const rosbag2_storage::TopicMetadata & tm) {
        return tm.name == topic_name;
      });
    if (it == metadata.end()) {
      RCLCPP_WARN(this->get_logger(), "Topic '%s' not found in the bag.", topic_name.c_str());
      handler.handler.reset(); // Remove handler if topic not found
    } else {
      RCLCPP_INFO(this->get_logger(), "Topic '%s' is available.", topic_name.c_str());
    }
  }

  // Read and process messages
  size_t total_messages = 0;
  while (reader.has_next()) {
    auto serialized_msg = reader.read_next();
    std::string topic = serialized_msg->topic_name;

    auto handler_it = handlers_.find(topic);
    if (handler_it != handlers_.end() && handler_it->second.handler) {
      size_t current_index = handler_it->second.current_index;

      // Find the sample interval for the topic
      auto topic_it = std::find_if(topics_.begin(), topics_.end(),
        [&topic](const TopicConfig & config) {
          return config.name == topic;
        });

      if (topic_it != topics_.end()) {
        size_t sample_interval = topic_it->sample_interval;  // Get the sample interval

        // Only write the message if it matches the sampling rate
        if (current_index % sample_interval == 0) {
          // Construct rclcpp::SerializedMessage from serialized_data
          rclcpp::SerializedMessage ser_msg;
          size_t buffer_length = serialized_msg->serialized_data->buffer_length;
          ser_msg.reserve(buffer_length);
          std::memcpy(ser_msg.get_rcl_serialized_message().buffer, 
                      serialized_msg->serialized_data->buffer, 
                      buffer_length);
          ser_msg.get_rcl_serialized_message().buffer_length = buffer_length;

          // Process the message
          handler_it->second.handler->process_message(ser_msg, topic, current_index);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "No configuration found for topic: %s", topic.c_str());
      }

      handler_it->second.current_index++;
      total_messages++;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Export completed. Total messages processed: %zu", total_messages);
  rclcpp::shutdown();
}

}  // namespace rosbag2_exporter

// Define the main function to initialize and run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exporter = std::make_shared<rosbag2_exporter::BagExporter>(rclcpp::NodeOptions());
  return 0;
}
