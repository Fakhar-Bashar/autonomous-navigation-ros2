# Computer Vision Line Following Robot

![Demo](../media/line_follower/line_follower_demo.gif)

[← Back to Portfolio](../README.md)

---

## 📋 Project Overview

Autonomous line following system using real-time computer vision and proportional feedback control. The robot detects and follows a black line on a white surface using camera-based vision processing.

---

## 🎯 Objectives

1. Implement real-time line detection using computer vision
2. Design proportional control system for smooth path following
3. Handle both straight and curved paths autonomously
4. Achieve stable 30 Hz processing rate
5. Demonstrate sensor-motor integration

---

## 🔧 Technical Implementation

### System Architecture
```
┌──────────────┐    Image     ┌──────────────┐    Twist    ┌──────────────┐
│   Camera     │──────────────>│ Line         │────────────>│   Motors     │
│  (30 Hz)     │   640×480    │ Follower     │  Velocity   │              │
└──────────────┘              └──────────────┘   Commands  └──────────────┘
                                     │
                                     │ cv_bridge
                                     ↓
                              ┌──────────────┐
                              │   OpenCV     │
                              │  Processing  │
                              └──────────────┘
```

### Image Processing Pipeline

**Step 1: Region of Interest (ROI)**
```python
height, width = image.shape[:2]
roi_height = height // 3
roi = image[height - roi_height:height, :]
```
- Focus on bottom third of image where line is visible
- Reduces processing time by 66%
- Improves noise immunity

**Step 2: Color Space Conversion**
```python
gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
```
- Convert from 3-channel BGR to 1-channel grayscale
- Simplifies subsequent processing
- Reduces computational load

**Step 3: Binary Thresholding**
```python
_, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
```
- Pixels < 50 → 255 (white)
- Pixels ≥ 50 → 0 (black)
- Inverted: line becomes white in binary image
- Threshold value tuned for optimal detection

**Step 4: Contour Detection**
```python
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```
- Find outlines of white regions
- RETR_EXTERNAL: only outer contours
- Identifies line boundary

**Step 5: Center Calculation**
```python
M = cv2.moments(largest_contour)
cx = int(M['m10'] / M['m00'])
```
- Calculate image moments
- Determine center of mass
- Extract X-coordinate of line center

### Control Algorithm

**Proportional Feedback Control:**
```python
# Calculate error
error = line_center_x - image_center_x

# Calculate turn rate (proportional to error)
angular_velocity = -error × gain

# Why negative?
# error > 0 (line right) → turn right (negative angular)
# error < 0 (line left) → turn left (positive angular)
```

**Mathematical Model:**
```
ω(t) = -Kp × e(t)

where:
  ω(t) = angular velocity (rad/s)
  Kp = proportional gain (0.005)
  e(t) = error (pixels)
```

**Control Loop:**
```
1. Measure: Detect line position
2. Compare: Calculate error from center
3. Compute: Proportional control law
4. Actuate: Send velocity command
5. Repeat: 30 times per second
```

---

## 📊 Performance Analysis

### Metrics

| Parameter | Value | Unit |
|-----------|-------|------|
| Processing Rate | 30 | Hz |
| Image Resolution | 640 × 480 | pixels |
| ROI Height | 160 | pixels |
| Latency | < 33 | ms/frame |
| Path Accuracy | ± 2 | cm |
| Linear Speed | 0.1 | m/s |
| Angular Gain | 0.005 | rad/(s·pixel) |

### Processing Breakdown

| Stage | Time | Percentage |
|-------|------|------------|
| Image Acquisition | 5 ms | 15% |
| ROI Extraction | 2 ms | 6% |
| Grayscale Conversion | 3 ms | 9% |
| Thresholding | 4 ms | 12% |
| Contour Detection | 8 ms | 24% |
| Moment Calculation | 5 ms | 15% |
| Control Computation | 1 ms | 3% |
| Command Publishing | 2 ms | 6% |
| **Total** | **30 ms** | **90%** |

(10% buffer for system overhead)

---

## 🛠️ Technologies Used

- **ROS2 Humble:** Robot middleware
- **OpenCV 4.5:** Computer vision library
- **cv_bridge:** ROS-OpenCV integration
- **Python 3.10:** Implementation language
- **NumPy:** Numerical computing
- **Gazebo 11:** Physics simulation
- **TurtleBot3 Waffle:** Robot platform (simulated)

---

## 🚀 Installation & Usage

### Prerequisites
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install python3-opencv
pip install --break-system-packages opencv-python numpy
```

### Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/Fakhar-Bashar/autonomous-navigation-ros2.git
cd ~/ros2_ws
colcon build --packages-select line_follower_pkg
source install/setup.bash
```

### Run
```bash
# Terminal 1: Launch simulation
export TURTLEBOT3_MODEL=waffle
ros2 launch line_follower_pkg professional_track.launch.py

# Terminal 2: Run line follower
ros2 run line_follower_pkg line_follower
```

---

## 🎓 Skills Demonstrated

### Computer Vision
- Image preprocessing and ROI selection
- Color space transformations
- Binary thresholding techniques
- Contour detection and analysis
- Moment-based center calculation
- Real-time vision processing constraints

### Control Theory
- Proportional feedback control design
- Error signal calculation
- Negative feedback loop implementation
- Parameter tuning methodology
- System stability analysis
- Real-time control at 30 Hz

### ROS2 Development
- Publisher-subscriber architecture
- Image message handling (sensor_msgs/Image)
- Velocity command publishing (geometry_msgs/Twist)
- cv_bridge integration
- Node lifecycle management
- Launch file configuration

### Software Engineering
- Modular code organization
- Real-time performance optimization
- Error handling and recovery
- Configurable parameters
- Inline documentation
- Git version control

---

## 🔮 Future Enhancements

**Immediate Improvements:**
- [ ] Add PID control (integral & derivative terms)
- [ ] Implement intersection detection
- [ ] Add speed adaptation based on curvature

**Advanced Features:**
- [ ] Multi-color line following
- [ ] Lane detection (two-line following)
- [ ] Decision-making at intersections
- [ ] Hardware deployment on physical TurtleBot3

---

## 📹 Demo Video

[Full Demo Video](../media/line_follower/line_follower_demo.mp4)

---

## 🏭 Industrial Applications

This project demonstrates skills applicable to:

**Warehouse Automation:**
- AGV path following on marked floors
- Automated material transport

**Manufacturing:**
- Seam tracking for welding robots
- Assembly line position tracking

**Quality Inspection:**
- Defect detection along production lines
- Automated visual inspection

---

## 📚 Key Learnings

1. **Real-time Constraints:** Vision processing must complete within frame period
2. **Parameter Tuning:** Gain values critically affect stability and responsiveness
3. **ROI Selection:** Processing only relevant image regions improves performance
4. **Control Theory:** Simple proportional control often sufficient for smooth tracking
5. **Sensor-Motor Integration:** Tight coupling between perception and action

---

## 📧 Contact

**Fakhar Bashar**  
Robotics Software Engineer

GitHub: [@Fakhar-Bashar](https://github.com/Fakhar-Bashar)  
Location: Kaiserslautern, Germany

---

[← Back to Portfolio](../README.md)
