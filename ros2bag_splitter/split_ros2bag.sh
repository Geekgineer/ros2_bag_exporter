#!/bin/bash

# 
#  Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
#  Date: 13.10.2024
# 

# ================================
# ROS2 Bag Splitting Script
# ================================
# This script splits a ROS2 bag file based on either maximum file size or maximum duration.
# The splitting can be configured to occur after a certain duration (in seconds) or
# once the bag file reaches a specified size (in bytes).
#
# Usage:
# ./split_ros2bag.sh input_bag_directory output_bag_directory --size max_size_in_bytes
# or
# ./split_ros2bag.sh input_bag_directory output_bag_directory --duration max_duration_in_seconds
#
# Arguments:
# 1. input_bag_directory   - The input ROS2 bag directory.
# 2. output_bag_directory  - The base name for the output ROS2 bag directories.
# 3. --size or --duration  - The splitting criterion.
# 4. max_value             - The value for the splitting criterion (size in bytes or duration in seconds).
#
# Example usage:
# ./split_ros2bag.sh my_input_bag my_output_bag --size 100000000  # Split every 100MB
# ./split_ros2bag.sh my_input_bag my_output_bag --duration 3600     # Split every hour
#
# =================================

# Exit script if any command fails, if any variable is unset, and if any command in a pipeline fails
set -euo pipefail

# Function to print usage instructions
print_usage() {
  echo "Usage: $0 input_bag_directory output_bag_directory --size max_size_in_bytes"
  echo "   or: $0 input_bag_directory output_bag_directory --duration max_duration_in_seconds"
  echo ""
  echo "Arguments:"
  echo "  input_bag_directory   - The input ROS2 bag directory."
  echo "  output_bag_directory  - The base name for the output ROS2 bag directories."
  echo "  --size                - Split based on maximum file size in bytes."
  echo "  --duration            - Split based on maximum duration in seconds."
  echo ""
  echo "Example:"
  echo "  $0 my_input_bag my_output_bag --size 100000000    # Split every 100MB"
  echo "  $0 my_input_bag my_output_bag --duration 3600       # Split every hour"
  exit 1
}

# Function to print error messages
error_exit() {
  echo "Error: $1" >&2
  exit 1
}

# Check if ros2 command is installed
if ! command -v ros2 &> /dev/null; then
  error_exit "'ros2' command not found. Please install ROS 2 and ensure 'ros2' is in your PATH."
fi

# Check if the required number of arguments are passed
if [ "$#" -lt 4 ] || [ "$#" -gt 4 ]; then
  print_usage
fi

# Assign arguments to variables
INPUT_BAG_DIR="$1"
OUTPUT_BAG_BASE="$2"
SPLIT_TYPE="$3"
SPLIT_VALUE="$4"

# Validate split type and value
if [ "$SPLIT_TYPE" == "--size" ]; then
  if ! [[ "$SPLIT_VALUE" =~ ^[0-9]+$ ]]; then
    error_exit "For --size, max_size_in_bytes must be a positive integer."
  fi
  SPLIT_CRITERION="size"
elif [ "$SPLIT_TYPE" == "--duration" ]; then
  if ! [[ "$SPLIT_VALUE" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    error_exit "For --duration, max_duration_in_seconds must be a positive number."
  fi
  SPLIT_CRITERION="duration"
else
  error_exit "Invalid split type. Use --size or --duration."
fi

# Check if input bag directory exists
if [ ! -d "$INPUT_BAG_DIR" ]; then
  error_exit "Input bag directory '$INPUT_BAG_DIR' does not exist."
fi

# Check if output bag directory already exists
if [ -e "$OUTPUT_BAG_BASE" ]; then
  error_exit "Output bag base directory '$OUTPUT_BAG_BASE' already exists. Please choose a different name or remove the existing directory."
fi

# Create a temporary YAML configuration file for ros2 bag convert
TEMP_YAML=$(mktemp /tmp/ros2bag_convert_XXXX.yaml)

# Function to clean up temporary files upon exit
cleanup() {
  rm -f "$TEMP_YAML"
}
trap cleanup EXIT

# Generate YAML configuration based on split criterion
if [ "$SPLIT_CRITERION" == "size" ]; then
  cat > "$TEMP_YAML" <<EOL
output_bags:
  - uri: ${OUTPUT_BAG_BASE}_%n
    storage_id: sqlite3
    max_bagfile_size: ${SPLIT_VALUE}
    max_bagfile_duration: 0
    compression_mode: none
    compression_format: none
    all_topics: true
EOL
elif [ "$SPLIT_CRITERION" == "duration" ]; then
  cat > "$TEMP_YAML" <<EOL
output_bags:
  - uri: ${OUTPUT_BAG_BASE}_%n
    storage_id: sqlite3
    max_bagfile_size: 0
    max_bagfile_duration: ${SPLIT_VALUE}
    compression_mode: none
    compression_format: none
    all_topics: true
EOL
fi

echo "Generated temporary YAML configuration for ros2 bag convert:"
cat "$TEMP_YAML"
echo "---------------------------------------------"

# Perform the bag conversion with splitting
echo "Starting ros2 bag conversion with splitting based on $SPLIT_CRITERION..."
ros2 bag convert "$INPUT_BAG_DIR" --output-options "$TEMP_YAML"

# Confirmation message
echo "Splitting complete. Output bags are prefixed with '$OUTPUT_BAG_BASE_'."
echo "Each split bag will have a unique numerical suffix (e.g., ${OUTPUT_BAG_BASE}_0, ${OUTPUT_BAG_BASE}_1, ...)."

