# 4iLab Hexacopter SLAM

A ROS-based Simultaneous Localization and Mapping (SLAM) system for hexacopter drones using Hector SLAM and RPLIDAR A3 sensor.

## Overview

This project implements a complete SLAM solution for hexacopter drones developed at 4i Labs, IIT Guwahati. The system enables real-time environment mapping and localization using a 2D LiDAR sensor mounted on a hexacopter platform. The implementation uses Hector SLAM algorithm which operates without odometry data, making it ideal for aerial platforms where wheel encoders are not available.

## Features

- **Real-time SLAM**: Live mapping and localization using Hector SLAM algorithm
- **LiDAR Integration**: Full support for RPLIDAR A3 with 16,000 samples/second
- **Drone-Optimized**: Configured specifically for hexacopter aerial platforms
- **ROS Integration**: Complete ROS workspace with launch files and configurations
- **Visualization**: Real-time map visualization through RViz
- **No Odometry Required**: Works without wheel encoders or IMU odometry

## Hardware Requirements

### Primary Components
- **Hexacopter Platform**: 6-rotor drone with payload capacity for LiDAR
- **RPLIDAR A3**: 360° laser scanner with 25m range
- **Flight Controller**: Compatible with MAVLink/MAVROS
- **Onboard Computer**: Linux system capable of running ROS

### Technical Specifications
- **LiDAR Range**: Up to 25 meters
- **Scan Rate**: 16,000 samples per second
- **Angular Resolution**: 0.225° - 0.45°
- **Communication**: USB interface for LiDAR

## Software Dependencies

### ROS Environment
```bash
# ROS Kinetic (recommended) or newer
sudo apt-get install ros-kinetic-desktop-full

# Additional ROS packages
sudo apt-get install ros-kinetic-hector-slam
sudo apt-get install ros-kinetic-rplidar-ros
```

### System Requirements
- **OS**: Ubuntu 16.04+ with ROS Kinetic/Melodic/Noetic
- **Python**: 2.7 or 3.x (depending on ROS version)
- **Build System**: catkin_make

## Installation

### 1. Create Catkin Workspace
```bash
mkdir -p ~/hexacopter_slam_ws/src
cd ~/hexacopter_slam_ws/
catkin_make
```

### 2. Clone Repository
```bash
cd ~/hexacopter_slam_ws/src
git clone https://github.com/sharma18b/4iLabHexacopter_SLAM.git
```

### 3. Build Workspace
```bash
cd ~/hexacopter_slam_ws/
catkin_make
source devel/setup.bash
```

### 4. Configure USB Permissions
```bash
# Grant permissions to LiDAR USB port
sudo chmod 777 /dev/ttyUSB0
```

## Usage

### Basic SLAM Operation

#### 1. Launch LiDAR Node
```bash
# Terminal 1: Start LiDAR data acquisition
roslaunch rplidar_ros view_rplidar_a3.launch
```

#### 2. Start SLAM Mapping
```bash
# Terminal 2: Launch Hector SLAM
roslaunch hector_slam_launch tutorial.launch
```

#### 3. Visualization
- RViz will automatically open displaying:
  - Real-time laser scan data
  - Generated occupancy grid map
  - Robot trajectory
  - Transform frames

### Advanced Configuration

#### Custom Launch Parameters
```xml
<!-- Modify scan topic for different LiDAR models -->
<param name="scan_topic" value="/scan"/>

<!-- Adjust map resolution -->
<param name="map_resolution" value="0.05"/>

<!-- Configure coordinate frames -->
<param name="base_frame" value="base_link"/>
<param name="odom_frame" value="base_link"/>
```

## System Architecture

### ROS Node Structure
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   RPLIDAR A3    │───▶│  Hector SLAM     │───▶│     RViz        │
│   /scan topic   │    │  /map topic      │    │  Visualization  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Transform      │    │  Map Server      │    │  Trajectory     │
│  Publisher      │    │  /map_metadata   │    │  Server         │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Key ROS Topics
- `/scan`: Raw LiDAR data (sensor_msgs/LaserScan)
- `/map`: Occupancy grid map (nav_msgs/OccupancyGrid)
- `/tf`: Transform tree for coordinate frames
- `/trajectory`: Robot path history

## Configuration Files

### Launch Files
- `view_rplidar_a3.launch`: LiDAR data acquisition
- `tutorial.launch`: Complete SLAM pipeline
- `mapping_default.launch`: Core mapping parameters

### Parameter Modifications
The system includes pre-configured parameters optimized for drone operation:

```yaml
# Frame configurations for aerial platform
base_frame: "base_link"
odom_frame: "base_link"
map_frame: "map"

# Mapping parameters
map_resolution: 0.05
map_size: 2048
map_start_x: 0.5
map_start_y: 0.5
```

## Troubleshooting

### Common Issues

#### LiDAR Connection Problems
```bash
# Check USB device recognition
lsusb | grep CP210

# Verify port permissions
ls -l /dev/ttyUSB*

# Reset USB permissions
sudo chmod 777 /dev/ttyUSB0
```

#### Transform Frame Errors
```bash
# Check transform tree
rosrun tf view_frames

# Verify static transforms
rosrun tf tf_echo base_link laser
```

#### Performance Optimization
- Reduce map resolution for faster processing
- Adjust scan frequency based on flight speed
- Monitor CPU usage during operation

### Debug Commands
```bash
# Monitor topics
rostopic list
rostopic echo /scan

# Check node status
rosnode list
rosnode info /hector_mapping

# Verify transforms
rosrun tf tf_monitor
```

## Integration with Flight Controller

### MAVROS Setup (Future Enhancement)
```bash
# Install MAVROS for PX4/ArduPilot integration
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras

# Configure for hexacopter
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600"
```

## File Structure

```
4iLabHexacopter_SLAM/
├── README.md                    # Project documentation
├── map_new_sac.mp4             # Demo video of SLAM operation
├── src/
│   └── RPLidar_Hector_SLAM/
│       ├── README.md           # Technical setup guide
│       ├── hector_slam/        # Hector SLAM package
│       │   ├── hector_mapping/
│       │   ├── hector_slam_launch/
│       │   └── [other modules]
│       └── rplidar_ros/        # RPLIDAR driver package
├── build/                      # Catkin build files
└── devel/                      # Development environment
```

## Performance Metrics

### Mapping Accuracy
- **Resolution**: 5cm per pixel (configurable)
- **Range**: Up to 25m radius
- **Update Rate**: 10-20 Hz depending on platform
- **Memory Usage**: ~100MB for typical indoor environments

### System Requirements
- **CPU**: Minimum dual-core ARM or x86
- **RAM**: 2GB minimum, 4GB recommended
- **Storage**: 1GB for workspace and maps

## Contributing

### Development Areas
- Enhanced drone-specific optimizations
- Integration with flight control systems
- Multi-floor mapping capabilities
- Real-time path planning integration
- Performance improvements for embedded systems

### Code Standards
- Follow ROS coding conventions
- Include comprehensive launch file documentation
- Test on actual hardware before submitting PRs

## Research Applications

This system has been used for:
- Indoor environment mapping with drones
- Search and rescue scenario mapping
- Autonomous navigation research
- Multi-robot SLAM studies

## License

This project is developed for research and educational purposes at 4i Labs, IIT Guwahati. Please refer to individual package licenses for specific terms.

## Acknowledgments

- **4i Labs, IIT Guwahati**: Research facility and project support
- **TU Darmstadt**: Original Hector SLAM algorithm
- **RoboPeak**: RPLIDAR hardware and drivers
- **ROS Community**: Framework and ecosystem support

## Contact

For technical questions and collaboration:
- **Organization**: 4i Labs, IIT Guwahati
- **Project**: Hexacopter SLAM Research
- **Repository**: [GitHub](https://github.com/sharma18b/4iLabHexacopter_SLAM)

## Demo

Check out `map_new_sac.mp4` for a demonstration of the system in action, showing real-time mapping of an indoor environment using the hexacopter platform.
