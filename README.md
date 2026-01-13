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

### 3. Install Required Packages

For the minimal simulator, you need:
```bash
sudo apt install ros-humble-ros-gz-sim
```

### 4. Run the Simulator

**Option A: Use the minimal simulator (Recommended)**
```bash
# Build robotverseny_description first (if not already built)
cd ~/ros2_ws
colcon build --packages-select robotverseny_description --symlink-install
source install/setup.bash

# Start the minimal simulator
ros2 launch ackermann_thesis minimal_sim.launch.py rviz:=true

# If you get OGRE rendering errors (common in WSL), you can try:
# - Make sure you have X11 forwarding set up
# - Or use a different display method
# - The bridge should still work even if Gazebo GUI fails
```

**Option B: Use the full robotverseny_gazebo24 simulator**
```bash
# If you have robotverseny_gazebo24 fully built
ros2 launch robotverseny_bringup roboworks.launch.py rviz:=true
```

**Note:** The test navigation node requires:
- `/scan` topic (LaserScan messages)
- TF transforms between `odom_combined`, `base_link`, and `laser` frames
- The simulator should be running before starting the navigation node

### 5. Run Navigation Nodes

**Option A: With Mock LiDAR (Recommended for Testing in WSL)**

If Gazebo has rendering issues, use the mock LiDAR publisher:

```bash
# Terminal 1: Start mock LiDAR publisher
source ~/ros2_ws/install/setup.bash
ros2 run ackermann_thesis mock_lidar_publisher

# Terminal 2: Run your navigation node
source ~/ros2_ws/install/setup.bash
ros2 run ackermann_thesis test_nav_node
```

The mock publisher creates:
- `/scan` topic with fake obstacle data
- TF transforms (`odom_combined` -> `base_link` -> `laser`)

**Option B: With Simulator**

```bash
# Terminal 1: Start simulator
source ~/ros2_ws/install/setup.bash
ros2 launch ackermann_thesis minimal_sim.launch.py rviz:=true

# Terminal 2: Run navigation node
source ~/ros2_ws/install/setup.bash
ros2 run ackermann_thesis test_nav_node
```

**Option C: Using Launch File**

```bash
# After simulator/mock publisher is running
source ~/ros2_ws/install/setup.bash
ros2 launch ackermann_thesis test_nav.launch.py
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
