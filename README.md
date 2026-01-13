# Ackermann Navigation Thesis

ROS 2 navigation system for Ackermann steering robot - Thesis project by Gaál Dominik.

## Overview

This repository contains the implementation of an autonomous path planning and navigation system for a Wheeltec Ackermann-steering robot using ROS 2 Humble. The project focuses on developing a complete navigation stack including environment sensing, state estimation, motion planning, and motion control.

## Repository Structure

```
ackermann_nav_thesis/
├── ackermann_thesis/          # ROS 2 package
│   ├── ackermann_thesis/      # Python source code
│   ├── launch/                # Launch files
│   ├── package.xml
│   └── setup.py
└── README.md
```

## Dependencies

### Required ROS 2 Packages
- ROS 2 Humble Hawksbill
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `visualization_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`

### External Dependencies
- **Simulator**: [`robotverseny_gazebo24`](https://github.com/robotverseny/robotverseny_gazebo24) - Gazebo simulation environment (optional, for full simulation)
- **Robot Model**: [`megoldas_sim24`](https://github.com/robotverseny/megoldas_sim24) - Wheeltec Ackermann robot model (used as reference, not modified)

## Setup Instructions

### 1. Clone Dependencies

```bash
cd ~/ros2_ws/src
git clone https://github.com/robotverseny/megoldas_sim24.git
```

**Optional - For Full Simulator:**
```bash
# Clone the simulator repository
git clone https://github.com/robotverseny/robotverseny_gazebo24.git
```

### 2. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Run the Simulator

Start the simulator using the method provided by `robotverseny_gazebo24`. 
The exact command depends on how the simulator package is structured in your setup.

**Note:** The test navigation node requires:
- `/scan` topic (LaserScan messages)
- TF transforms between `odom_combined`, `base_link`, and `laser` frames
- The simulator should be running before starting the navigation node

### 4. Run Navigation Nodes

**Using launch file:**
```bash
# After simulator is running (or in another terminal)
source ~/ros2_ws/install/setup.bash
ros2 launch ackermann_thesis test_nav.launch.py
```

**Or run directly:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ackermann_thesis test_nav_node
```

## Architecture

The system is designed with the following components:

1. **SensorNode**: Handles sensor data (LiDAR, odometry)
2. **ObstacleDetector**: Processes sensor data to detect obstacles
3. **LocalPlanner**: Implements local path planning with obstacle avoidance
4. **AckermannController**: Converts planned paths to Ackermann-compatible control commands

## Development Status

- [x] Repository setup
- [x] ROS 2 package structure
- [ ] Sensor integration
- [ ] Board model (local occupancy grid)
- [ ] Base navigation algorithm
- [ ] Ackermann controller
- [ ] Testing and validation

## License

[To be determined]

## Author

Gaál Dominik - Mérnökinformatikus BSc
