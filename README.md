# Livox ROS Driver 2 - Humble Version

Livox ROS Driver 2 is the 2nd-generation driver package used to connect LiDAR products produced by Livox, specifically designed for ROS2 Humble.

  **Note :**

  As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## 1. Preparation

### 1.1 OS requirements

  * Ubuntu 22.04 for ROS2 Humble;

  **Tips:**

  Colcon is a build tool used in ROS2.

  How to install colcon: [Colcon installation instructions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

### 1.2 Install ROS2 Humble

For ROS2 Humble installation, please refer to:
[ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Desktop-Full installation is recommend.

## 2. Build & Run Livox ROS Driver 2

### 2.1 Clone Livox ROS Driver 2 source code:

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

  **Note :**

  Be sure to clone the source code in a '[work_space]/src/' folder (as shown above), otherwise compilation errors will occur due to the compilation tool restriction.

### 2.2 Build & install the Livox-SDK2

  **Note :**

  Please follow the guidance of installation in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)

### 2.3 Build the Livox ROS Driver 2:

**Recommended build method with symlink-install:**

```shell
source /opt/ros/humble/setup.sh
colcon build --packages-select livox_ros_driver2 --symlink-install
```

Or build the entire workspace with symlink-install:

```shell
source /opt/ros/humble/setup.sh
colcon build --symlink-install
```

**Why use --symlink-install?**

The `--symlink-install` option creates symbolic links instead of copying files during installation. This means:
- Changes to source code are immediately reflected without recompilation
- Faster development cycle
- Saves disk space
- No need to rebuild after code modifications

### 2.4 Run Livox ROS Driver 2:

```shell
source install/setup.sh
ros2 launch livox_ros_driver2 [launch file]
```

in which,  

* **[launch file]** : is the ROS2 launch file you want to use; the 'launch_ROS2' folder contains several launch samples for your reference.

A rviz launch example for MID360 LiDAR would be:

```shell
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

## 3. MID360 LiDAR Configuration and Usage

### 3.1 MID360 Launch Files

Launch files for MID360 LiDAR are in the "ws_livox/src/livox_ros_driver2/launch_ROS2" directory:

| launch file name          | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| rviz_MID360_launch.py        | Connect to MID360 LiDAR device<br>Publish pointcloud2 format data <br>Autoload rviz|
| msg_MID360_launch.py          | Connect to MID360 LiDAR device<br>Publish livox customized pointcloud data |

### 3.2 MID360 Configuration File Setup

The MID360 LiDAR configuration is stored in `config/MID360_config.json`. You need to modify this file to match your network setup.

**Step 1: Find your MID360 LiDAR IP address**

First, you need to find the IP address of your MID360 LiDAR. You can:
- Check the LiDAR's label for the default IP
- Use Livox Viewer software to discover the device
- Check your network router's DHCP client list

**Step 2: Configure the host IP**

Edit `config/MID360_config.json` and modify the host IP addresses:

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.50",  # Change this to your computer's IP
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.50",   # Change this to your computer's IP
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50", # Change this to your computer's IP
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.50",  # Change this to your computer's IP
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # Change this to your MID360's IP address 1**
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

**Step 3: Configure your computer's network**

Make sure your computer is on the same network as the MID360 LiDAR. You may need to:

1. Set a static IP on your computer (e.g., 192.168.1.5)
2. Connect both devices to the same network switch/router
3. Ensure firewall allows the required ports

**Step 4: Test the connection**

After configuration, test the connection:

```shell
# Ping the LiDAR to check network connectivity
ping 192.168.1.100

# Run the driver
source install/setup.sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 3.3 MID360 Parameter Configuration

The main parameters for MID360 LiDAR configuration:

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Set the frequency of point cloud publish <br>Floating-point data type, recommended values 5.0, 10.0, 20.0, 50.0, etc. The maximum publish frequency is 100.0 Hz.| 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTLT) pointcloud format<br>1 -- Livox customized pointcloud format<br>2 -- Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library | 0       |

### 3.4 MID360 Data Format

MID360 LiDAR provides the following data formats:

1. **Livox pointcloud2 (PointXYZRTLT) format:**

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```

2. **Livox customized data package format:**

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

3. **Standard pointcloud2 (pcl :: PointXYZI) format:**

Please refer to the pcl :: PointXYZI data structure in the point_types.hpp file of the PCL library.

## 4. Troubleshooting MID360 Issues

### 4.1 Network Connection Issues

**Problem:** Cannot connect to MID360 LiDAR

**Solutions:**
1. Check if the LiDAR IP address is correct in the config file
2. Verify your computer's IP is in the same subnet
3. Test network connectivity with ping
4. Check firewall settings
5. Ensure the LiDAR is powered on and connected to the network

### 4.2 No Point Cloud Data

**Problem:** Driver runs but no point cloud data is published

**Solutions:**
1. Check the "Global Options - Fixed Frame" field in RViz, set to "livox_frame"
2. Verify the PointCloud2 option is enabled in RViz
3. Check if the LiDAR is scanning (look for moving parts)
4. Verify the configuration file paths in the launch file

### 4.3 Library Issues

**Problem:** Cannot open shared object file "liblivox_sdk_shared.so"

**Solution:** Add '/usr/local/lib' to the LD_LIBRARY_PATH:

```shell
# For current terminal
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

# For permanent setup
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib' >> ~/.bashrc
source ~/.bashrc
```

## 5. Supported LiDAR list

* HAP
* Mid360
* (more types are comming soon...)

## 6. FAQ

### 6.1 How to change MID360 IP address?

You can change the MID360 IP address by:
1. Using Livox Viewer software to configure the device
2. Modifying the "ip" field in the lidar_configs section of the config file
3. Ensuring the host_net_info IP addresses match your computer's network configuration

### 6.2 Why use --symlink-install for development?

The `--symlink-install` option creates symbolic links instead of copying files, which means:
- Code changes are immediately available without recompilation
- Faster development cycle
- Saves disk space
- Ideal for iterative development and testing

### 6.3 How to check if MID360 is working properly?

1. Check the LiDAR's LED indicators
2. Use Livox Viewer to verify the device is detected
3. Monitor the ROS2 topics for data:
   ```shell
   ros2 topic list
   ros2 topic echo /livox/lidar
   ```
4. Check RViz for point cloud visualization

### 6.4 What are the default network settings for MID360?

- Default LiDAR IP: 192.168.1.100
- Command port: 56100
- Point data port: 56300
- IMU data port: 56400
- Log data port: 56500
