cmake_minimum_required(VERSION 3.5)
project(ros2_bag_exporter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable CMP0074 to allow <PackageName>_ROOT variables
cmake_policy(SET CMP0074 NEW)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ament_index_cpp_INCLUDE_DIRS} 
)

add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(bag_exporter src/bag_exporter.cpp)

# Link libraries
ament_target_dependencies(bag_exporter
  ament_cmake
  ament_index_cpp
  rclcpp
  rosbag2_cpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  OpenCV
  yaml-cpp
)

target_link_libraries(bag_exporter
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # ${YAML_CPP_LIBRARIES}
  ${ament_index_cpp_INCLUDE_DIRS} 
  yaml-cpp
  ament_index_cpp::ament_index_cpp

)



# Install executable
install(TARGETS
  bag_exporter
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install configuration files (ensure config directory and files exist)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/
  )
else()
  message(WARNING "Config directory does not exist in ${CMAKE_CURRENT_SOURCE_DIR}")
endif()


ament_package()
