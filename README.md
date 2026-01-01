# ROS2 Robotics Portfolio

Professional robotics software engineering projects demonstrating autonomous systems, computer vision, and navigation capabilities.

> 🎯 **Seeking robotics software engineering opportunities in Germany**

---

## 🚀 Featured Projects

### 1. Autonomous Navigation Robot with SLAM
![Navigation Demo](media/house/house_navigation.gif)

**Multi-environment autonomous mobile robot with real-time mapping and path planning**

**Key Features:**
- ✅ Real-time SLAM mapping with SLAM Toolbox
- ✅ Autonomous navigation using Nav2 stack
- ✅ Dynamic obstacle avoidance
- ✅ Multi-room path planning
- ✅ Tested in residential and warehouse environments

**Technologies:** ROS2 Humble • Navigation2 • SLAM Toolbox • Gazebo • Python • C++

**[📂 View Project Details →](docs/navigation_project.md)**

---

### 2. Computer Vision Line Following Robot
![Line Follower](media/line_follower/line_follower_demo.gif)

**Vision-based autonomous path following with real-time image processing**

**Key Features:**
- ✅ Real-time line detection at 30 Hz
- ✅ OpenCV image processing pipeline
- ✅ Proportional feedback control
- ✅ Handles curved and straight paths
- ✅ Continuous autonomous operation

**Technologies:** ROS2 • OpenCV • cv_bridge • Python • Computer Vision

**[📂 View Project Details →](docs/line_follower_project.md)**

---

## 💻 Technical Skills

### Robotics & Autonomous Systems
- **ROS2 Development:** Nodes, Topics, Services, Actions, Launch Files
- **Navigation:** SLAM, Localization (AMCL), Path Planning (A*, DWA)
- **Computer Vision:** OpenCV, Image Processing, Real-time Detection
- **Control Systems:** Proportional Control, Feedback Loops, Parameter Tuning
- **Sensor Fusion:** LIDAR, Camera, IMU, Wheel Odometry

### Programming & Tools
- **Languages:** Python (Primary), C++ (Familiar)
- **Frameworks:** ROS2 Humble, Navigation2, OpenCV
- **Simulation:** Gazebo 11
- **Version Control:** Git, GitHub
- **OS:** Ubuntu 22.04, Linux

### Algorithms Implemented
- Simultaneous Localization and Mapping (SLAM)
- A* Global Path Planning
- Dynamic Window Approach (DWA) Local Planning
- Image Thresholding and Contour Detection
- Proportional Control Systems
- Costmap-based Obstacle Representation

---

## 📊 Portfolio Statistics

| Metric | Value |
|--------|-------|
| Complete Projects | 2 |
| Total Code Lines | 1,000+ |
| Sensors Integrated | 4 (Camera, LIDAR, IMU, Odometry) |
| Environments Tested | 3 (House, Warehouse, Track) |
| Processing Rate | 30 Hz (real-time) |
| Success Rate | 95%+ |

---

## 🎯 About Me

Robotics software engineer with hands-on experience in autonomous navigation, computer vision, and mobile robot control. Passionate about building intelligent systems that solve real-world problems.

**🔧 What I Bring:**
- Strong foundation in ROS2 and autonomous systems
- Practical computer vision implementation skills
- Understanding of control theory and sensor fusion
- Ability to learn quickly and solve complex problems
- Clean, documented code with best practices

**📍 Location:** Kaiserslautern, Germany  
**💼 Status:** Actively seeking robotics software engineering roles  
**🌍 Work Authorization:** EU

---

## 🚀 Quick Start

### Clone Repository
```bash
git clone https://github.com/Fakhar-Bashar/autonomous-navigation-ros2.git
cd autonomous-navigation-ros2
```

### Setup Environment
```bash
# Ensure ROS2 Humble is installed
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### Run Navigation Demo
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=True \
    map:=./maps/house/house_map.yaml
```

### Run Line Follower Demo
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch line_follower_pkg professional_track.launch.py
ros2 run line_follower_pkg line_follower
```

---

## 📂 Repository Structure
```
autonomous-navigation-ros2/
├── docs/                          # Project documentation
│   ├── navigation_project.md
│   └── line_follower_project.md
├── media/                         # Demo videos and images
│   ├── house/
│   ├── warehouse/
│   └── line_follower/
├── maps/                          # SLAM-generated maps
│   ├── house/
│   └── warehouse/
└── README.md                      # This file
```

---

## 📹 Demo Videos

**Navigation System:**
- [House Environment Navigation](media/house/house_navigation.mp4)
- [Warehouse Navigation](media/warehouse/warehouse_demo.mp4)

**Line Following:**
- [Continuous Track Following](media/line_follower/line_follower_demo.mp4)

---

## 🎓 Learning Journey

These projects represent my journey in robotics development:

**Phase 1:** Understanding ROS2 fundamentals  
**Phase 2:** Implementing SLAM and navigation  
**Phase 3:** Adding computer vision capabilities  
**Phase 4:** System integration and optimization  

**Key Takeaways:**
- Real-time systems require careful optimization
- Parameter tuning is crucial for robot performance
- Simulation accelerates development and testing
- Clean architecture enables easier debugging
- Documentation is essential for maintainability

---

## 🤝 Connect With Me

**📧 Email:** fbqazi786@gmail.com  
**💼 LinkedIn:** https://www.linkedin.com/in/muhammad-fakhar-ul-bashar-4494641b7/  
**🐙 GitHub:** [@Fakhar-Bashar](https://github.com/Fakhar-Bashar)  
**📍 Location:** Kaiserslautern, Germany

---

## 📝 License

MIT License - Feel free to learn from this code!

---

## 🌟 Acknowledgments

Built as part of my robotics software engineering portfolio. These projects demonstrate practical skills in:
- Warehouse automation and AGV navigation
- Industrial line tracking systems
- Autonomous mobile robotics
- Computer vision for robotics

**Applicable Industries:** Warehouse Automation • Industrial Robotics • Autonomous Vehicles • Service Robotics

---

<div align="center">

### ⭐ If you find this repository helpful, please star it!

**Open to robotics software engineering opportunities in Germany** 🇩🇪

</div>

---

**Tags:** `ros2` `robotics` `autonomous-navigation` `slam` `computer-vision` `opencv` `python` `mobile-robotics` `path-planning` `gazebo` `navigation2`
