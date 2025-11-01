# Autonomous Navigation Robot - ROS2

Multi-environment autonomous navigation system with real-time SLAM mapping and intelligent path planning.

## 🎥 Demo: House Environment Navigation

![House Navigation](media/house/house_navigation.gif)

*Robot autonomously navigating through residential environment, avoiding furniture and traversing multiple rooms*

## 🎯 Overview
Autonomous mobile robot navigation system built with ROS2 Humble, featuring real-time SLAM mapping, autonomous path planning, and dynamic obstacle avoidance. Successfully tested in complex indoor environments including residential and warehouse settings.

## ✨ Key Features
- **🗺️ Real-time SLAM Mapping** - Simultaneous Localization and Mapping using SLAM Toolbox
- **🎯 Autonomous Navigation** - Goal-based navigation with Nav2 stack
- **🏠 Multi-Environment Support** - Tested in house and warehouse scenarios
- **🚧 Dynamic Obstacle Avoidance** - Real-time collision prevention
- **🚪 Multi-Room Navigation** - Complex indoor environment traversal
- **💻 Programmatic Control** - Python API for autonomous missions
- **📍 Multi-Waypoint Planning** - Sequential goal achievement
- **📊 Real-time Visualization** - RViz monitoring and debugging

## 🛠️ Technology Stack

**Core Technologies:**
- ROS2 Humble
- Nav2 Navigation Stack
- SLAM Toolbox
- Gazebo 11
- Python 3.10
- Ubuntu 22.04

**Key Algorithms:**
- SLAM for mapping and localization
- A* / Dijkstra for global path planning
- DWA (Dynamic Window Approach) for local planning
- Costmap-based obstacle representation

## 🚀 Quick Start

### Prerequisites
```bash
# Ubuntu 22.04 with ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### Installation
```bash
# Install dependencies
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-turtlebot3-navigation2

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/Fakhar-Bashar/autonomous-navigation-ros2.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 📖 Usage

### House Environment

#### Step 1: Launch Simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

#### Step 2: Launch Navigation (in new terminal)
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=True \
    map:=$(pwd)/maps/house/house_map.yaml
```

#### Step 3: Navigate in RViz
1. **Set Initial Pose:** Click "2D Pose Estimate" and place on map
2. **Send Goal:** Click "Nav2 Goal" and select destination
3. **Watch:** Robot navigates autonomously!

### Building New Maps (SLAM)
```bash
# Terminal 1: Launch environment
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# Terminal 2: Start SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Visualize
rviz2

# Terminal 4: Manual control
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 5: Save map after exploring
cd ~/maps
ros2 run nav2_map_server map_saver_cli -f my_map
```

## 📊 Performance

| Metric | House Environment |
|--------|-------------------|
| Map Build Time | ~3 minutes |
| Navigation Success Rate | 95%+ |
| Avg Path Planning Time | 0.4s |
| Obstacle Detection Range | 3.5m |
| Replanning Speed | Real-time |

## 🎓 Technical Skills Demonstrated

**ROS2 Expertise:**
- Node architecture and communication
- Topics, services, and actions
- Launch file configuration
- Parameter management

**Navigation & Planning:**
- SLAM algorithms
- A* path planning
- Dynamic Window Approach
- Costmap generation
- TF coordinate transformations

**Robotics Fundamentals:**
- Sensor fusion (LIDAR, odometry, IMU)
- Motion control
- Localization techniques
- Obstacle avoidance

**Software Engineering:**
- Python/C++ development
- Git version control
- Documentation
- System integration

## 📁 Repository Structure
```
autonomous-navigation-ros2/
├── maps/
│   └── house/
│       ├── house_map.yaml
│       └── house_map.pgm
├── media/
│   └── house/
│       ├── house_navigation.gif
│       └── house_navigation.mp4
├── src/
├── launch/
├── config/
├── docs/
└── README.md
```

## 🔮 Future Enhancements
- [ ] Multi-robot fleet coordination
- [ ] 3D navigation (multi-floor)
- [ ] Voice command interface
- [ ] Machine learning path optimization
- [ ] Deployment on physical TurtleBot3
- [ ] Integration with real sensors
- [ ] Custom world creation

## 📈 Development Journey

**Week 1:** SLAM implementation and mapping
**Week 2:** Autonomous navigation and path planning
**Week 3:** Multi-environment testing and optimization

## 🎯 Project Goals Achieved
- ✅ Real-time SLAM mapping
- ✅ Autonomous goal-based navigation
- ✅ Dynamic obstacle avoidance
- ✅ Multi-environment testing
- ✅ Professional documentation
- ✅ Demo videos and visualization

## 🤝 Collaboration
Developed in collaboration with [Mansi Gagaliya](https://github.com/Mansi-Gagaliya)

## 📧 Contact

**Fakhar Bashar**  
Robotics Software Engineer

- 🔗 GitHub: [@Fakhar-Bashar](https://github.com/Fakhar-Bashar)
- 💼 LinkedIn: https://www.linkedin.com/in/muhammad-fakhar-ul-bashar-4494641b7/
- 📧 Email: fbqazi786@gmail.com
- 📍 Location: Kaiserslautern, Germany
- 🎯 **Open to robotics/autonomous systems opportunities in Germany**

## 📝 License
MIT License

---

### 🌟 If you find this project useful, please star this repository!

**Tags:** `ros2` `robotics` `autonomous-navigation` `slam` `path-planning` `gazebo` `python` `cpp` `nav2` `mobile-robotics`
