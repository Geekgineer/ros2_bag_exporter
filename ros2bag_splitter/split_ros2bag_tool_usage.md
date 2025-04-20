
# ROS2 Bag Splitting Tool

## Overview
This tool script splits a ROS2 bag file based on either maximum file size or maximum duration. The splitting can be configured to occur after a certain duration (in seconds) or once the bag file reaches a specified size (in bytes).

## Usage
```bash
./split_ros2bag.sh input_bag_directory output_bag_directory --size max_size_in_bytes
or
./split_ros2bag.sh input_bag_directory output_bag_directory --duration max_duration_in_seconds
```

### Arguments
1. `input_bag_directory`   - The input ROS2 bag directory.
2. `output_bag_directory`  - The base name for the output ROS2 bag directories.
3. `--size` or `--duration` - The splitting criterion.
4. `max_value`             - The value for the splitting criterion (size in bytes or duration in seconds).

### Example Usage
```bash
./split_ros2bag.sh my_input_bag my_output_bag --size 100000000  # Split every 100MB
./split_ros2bag.sh my_input_bag my_output_bag --duration 3600     # Split every hour
```

## Usage Examples
### Splitting by Maximum File Size
To split an input ROS2 bag directory (my_input_bag) into multiple bags, each with a maximum size of 100MB:
```bash
./split_ros2bag.sh my_input_bag my_output_bag --size 100000000
```
Output:
- `my_output_bag_0` (≤ 100MB)
- `my_output_bag_1` (≤ 100MB)
- ...and so on.

### Splitting by Maximum Duration
To split an input ROS2 bag directory (my_input_bag) into multiple bags, each with a maximum duration of 1 hour (3600 seconds):
```bash
./split_ros2bag.sh my_input_bag my_output_bag --duration 3600
```
Output:
- `my_output_bag_0` (≤ 1 hour)
- `my_output_bag_1` (≤ 1 hour)
- ...and so on.

## Prerequisites
Before using the script, ensure that your system meets the following requirements:
- **Operating System**: Unix-like (e.g., Linux, macOS)
- **ROS 2 Installation**: ROS 2 (e.g., Humble) must be installed and properly configured.
- **rosbag2 Utility**: The ros2 command-line tool should be accessible in your system's PATH.
- **Bash Shell**: The script is written for Bash; ensure you have Bash installed.

## Installation
### Clone or Download the Script
You can clone the repository or download the script directly.
```bash
git clone https://github.com/Geekgineer/ros2_bag_exporter
cd ros2bag_splitter
```
Alternatively, download the `split_ros2bag.sh` script and place it in your desired directory.

### Make the Script Executable
Change the script's permissions to make it executable.
```bash
chmod +x split_ros2bag.sh
```

### Verify Dependencies
Ensure that ros2 is installed and accessible.
```bash
ros2 --version
```
If the command returns the version information, you're good to go. Otherwise, install ROS 2 and ensure ros2 is in your PATH.

## Notes
- **Storage Formats**: By default, the script uses the sqlite3 storage format. If you prefer to use a different storage format (e.g., mcap), modify the storage_id field in the YAML generation section of the script.
- **YAML Configuration**: The script generates a temporary YAML file to configure the ros2 bag convert command. This file is automatically deleted upon script completion.
- **Output Bag Naming**: The output bags are named using the base name provided (output_bag_directory) followed by an underscore and a numerical suffix (e.g., output_bag_0, output_bag_1, etc.).
- **Splitting Constraints**: When specifying the --size option, ensure that the size is greater than the minimum allowed by rosbag2 (if any). Similarly, when using the --duration option, provide a positive number representing seconds.
- **Storage Options**: For advanced storage configurations (e.g., compression), you can modify the YAML generation section to include additional parameters as supported by rosbag2.

## Potential Enhancements
- **Support for Compression**: Integrate compression options within the YAML configuration to allow splitting and compression simultaneously.
- **Dynamic Storage Format Selection**: Allow users to specify the desired storage_id as an additional argument to the script.
- **Logging**: Implement logging mechanisms to capture detailed execution logs for debugging purposes.
- **Parallel Processing**: For large bags, consider adding parallel processing capabilities to speed up the conversion and splitting process.

Feel free to customize this script further to fit specific needs or to integrate additional features as required. If you encounter any issues or have suggestions for improvements, consider contributing to the repository or reaching out for support.