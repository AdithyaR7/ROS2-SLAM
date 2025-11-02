# ROS2 SLAM with Object Detection and a Semantic Map 
Autonomous robot navigation system that builds semantic maps by fusing 2D LiDAR SLAM with real-time object detection and depth sensing. 
(pictures and video coming soon)

## Overview
This project extends traditional geometric SLAM by adding a **semantic layer** - the robot doesn't just map walls and obstacles, it identifies and remembers objects like chairs, tables, and other items. Navigate using natural commands like "go to the chair" instead of coordinates, enabling more intuitive human-robot interaction.

## How It Works
The system follows this pipeline:

1. **Geometric Mapping** - 2D LiDAR with slam_toolbox builds the base occupancy grid map
2. **Object Detection** - YOLOv10 runs continuously on RGB camera feed to detect objects
3. **Depth Integration** - RealSense depth camera provides 3D position for each detection
4. **Semantic Mapping** - Detected objects are transformed to map frame, deduplicated, and stored with their class labels and positions
5. **Semantic Navigation** - Natural language commands query the semantic map and send navigation goals to Nav2

During exploration, the robot simultaneously builds its geometric map while detecting objects with YOLO at 30 FPS. Each detection's 2D bounding box is combined with depth data to compute 3D positions in the map frame using camera intrinsics and TF transforms. Objects are deduplicated using a distance threshold to prevent multiple entries for the same item. This creates a persistent semantic layer - objects remain in memory even when out of view, enabling navigation to previously seen locations. Commands like "go to chair" query this semantic map, select an appropriate target (e.g., nearest object), and send the goal to Nav2 for autonomous path planning and execution.

## World & Robot
**Robot Configuration:**
- Differential drive base with caster wheel
- 2D 360° LiDAR (Hokuyo simulation) for SLAM
- RealSense D435 depth camera (simulated, 640x480 RGB-D) for object detection and depth measurements
- Compact indoor navigation platform - pre-made environment for proof of concept

**Simulation Environment:**
Modified TurtleBot3 house world with closed environment - the original door opening has been sealed with a wall to create a fully enclosed space. This ensures the robot can build a complete map without boundary issues and provides a realistic indoor setting with furniture and obstacles at appropriate scale for the robot's camera height.

<div align="center">
<img src="images/robot_gazebo.png" width="600" />
<p><i>Robot in modified TurtleBot3 house environment</i></p>
</div>

## Tools & Technologies

**Core Stack:**
- **ROS2 Iron** - Robot middleware and communication
- **Gazebo** - Physics simulation and sensor simulation
- **Nav2** - Navigation framework with path planning and control
- **slam_toolbox** - 2D LiDAR SLAM for geometric mapping
- **m-explore-ros2** - Frontier-based autonomous exploration
- **custom pacakges** - main nodes and logic to pull things together (see below)

**(custom) - vision_pkg** - Object detection and 3D position estimation
- Subscribes to RGB, depth, and camera info topics
- Runs YOLOv10 inference with GPU acceleration
- Converts 2D bounding boxes to 3D map coordinates using depth + camera intrinsics + TF
- Publishes `DetectionArray` messages containing detected objects with positions
- Provides annotated image visualization with bounding boxes

**(custom) - semantic_planner_pkg** - Semantic map maintenance and navigation interface
- Subscribes to detection arrays and maintains persistent semantic map
- Performs spatial deduplication (distance threshold) to prevent duplicate object entries
- Filters detections by confidence threshold
- Publishes `MarkerArray` for RViz visualization of semantic objects
- Parses natural language navigation commands
- Queries semantic map for target objects and sends goals to Nav2

**Perception:**
- **YOLOv10** - Real-time object detection
- **PyTorch** with CUDA - GPU-accelerated inference
- **OpenCV** - Image processing and coordinate transforms
- **cv_bridge** - ROS-OpenCV integration

## Semantic Map Structure
The semantic layer stores detected objects with:
- **Class name** - YOLO category (chair, table, bottle, etc.)
- **3D position** - (x, y, z) coordinates in map frame
- **Confidence** - Detection confidence score
- **Unique ID** - Prevents duplicate entries
- **Timestamp** - When object was first detected

Objects persist in memory even when not visible, enabling navigation to previously seen items.

<div align="center">
<img src="images/semantic_map_rviz.png" width="600" />
<p><i>RViz visualization showing geometric map with semantic object markers</i></p>
</div>

## Installation

### Prerequisites
- Ubuntu 22.04
- ROS2 Iron
- Python 3.10
- NVIDIA GPU with CUDA (recommended for real-time YOLO at ~30 fps)

### Clone Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [YOUR_REPO_URL]
cd ~/ros2_ws
```

### Install ROS2 Dependencies and Packages
```bash
sudo apt update
sudo apt install ros-iron-gazebo-ros-pkgs ros-iron-nav2-bringup \
  ros-iron-slam-toolbox ros-iron-cv-bridge \
  ros-iron-visualization-msgs ros-iron-tf-transformations

# Clone exploration package for autonomous mapping
cd ~/ros2_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
```

### Install Python Dependencies
```bash
# Downgrade NumPy for ROS2 compatibility
pip3 install "numpy<2"

# Install PyTorch with CUDA support
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu126

# Install computer vision packages
pip3 install ultralytics opencv-python transforms3d

# Note: If using conda, install these in system Python instead to avoid ROS2 conflicts
```

### Build Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running the Project

### Launch Everything
Start the complete system with a single launch file:
```bash
ros2 launch vehicle_bringup vehicle_yolo_launch.py
```

This brings up:
- Gazebo with modified TurtleBot3 house environment
- Robot spawn with differential drive controller
- slam_toolbox for 2D SLAM mapping
- Nav2 navigation stack with path planning
- Object detection node (YOLOv10 + depth processing)
- Semantic planner node (map maintenance + navigation interface)
- RViz with preconfigured displays

<div align="center">
<img src="images/rviz_gazebo_sidebyside.png" width="800" />
<p><i>Side-by-side view: Gazebo simulation (left) and RViz SLAM map (right)</i></p>
</div>

**Configuration Files:**
Parameters are managed through YAML files:
- **Nav2, SLAM, Semantic Planner, RViz:** `vehicle_bringup/config/*.yaml` and `vehicle_bringup/config/*.rviz`
- **explore_lite:** `m-explore-ros2/explore/config/params.yaml`

**RViz Settings:**
Load saved RViz configuration to view all displays (map, camera feeds, annotated feed, semantic markers, paths, etc.)

### Building the Map

**Option 1: Autonomous Exploration**
Launch explore_lite for frontier-based autonomous mapping:
```bash
ros2 launch explore_lite explore.launch.py use_sim_time:=true      # Run in separate terminal
```

The robot autonomously explores and builds the map.

<div align="center">
<img src="images/explore_lite_mapping.gif" width="600" />
<p><i>Autonomous mapping with explore_lite frontier exploration</i></p>
</div>

**Option 2: Manual Control**
Drive the robot manually using keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard      # Run in separate terminal
```
or simply click on the 2D Goal Pose button in Rviz and then a space on the map to move the robot there (easiest method to test)
<div align="center">
<img src="images/manual_mapping.gif" width="600" />
<p><i>Manual mapping with 2DGoalPose through Rviz</i></p>
</div>

<div align="center">
<img src="images/yolo_depth_detection.png" width="800" />
<p><i>Left: YOLO detections with bounding boxes. Right: Corresponding depth map. Bottom: RViz MarkerArray showing 3D positions in map frame</i></p>
</div>

### Navigation Commands
Once mapping is complete or as objects have been detected and stored, send semantic navigation commands:
```bash
# Navigate to detected objects
ros2 topic pub /navigation_command std_msgs/String "data: 'go to chair'" --once

ros2 topic pub /navigation_command std_msgs/String "data: 'go to table'" --once
```

If multiple instances of an object exist, the robot navigates to the closest one.

### RViz Visualization
RViz displays:
- **Map** - 2D occupancy grid with cost map from SLAM
- **Robot Model** - TF tree and sensor frames
- **Camera Feeds** - Raw and annotated images with detections (rgb and depth maps)
- **Semantic Markers** - Labeled array markers at object positions in saved semantic map and current visible objects
- **Navigation** - Global and local paths from Nav2

## Troubleshooting

**NumPy version conflicts:**
```bash
pip3 install "numpy<2" "opencv-python<4.10"
```

**CUDA not detected:**
```bash
python3 -c "import torch; print(torch.cuda.is_available())"
nvidia-smi
```

**TF transform errors:**
Check TF tree:
```bash
ros2 run tf2_tools view_frames
```

**No objects detected:**
- Verify camera topics: `ros2 topic list | grep camera`
- Check YOLO device log for GPU usage
- Adjust world lighting/object placement

## Future Extensions

**Short-term:**
- [ ] Create [MBOT](https://mbot.robotics.umich.edu/docs/hardware/classic/building/) URDF for better visuals and recreatable hardware integration
- [ ] Swap out vehicle for more realistic car-based model and driving mechanics to simulate an autonomous car as a separate test case
- [ ] Confidence and recency-based object filtering
- [ ] Save/load semantic maps for persistent memory across sessions

**Medium-term:**
- [ ] Periodic 360° rotations for complete semantic coverage (rgbd based, with existing LiDAR SLAM)
- [ ] Learning based method for depth-aware object segmentation (through rgbd or lidar) for accurate 3D positioning in semantic map. Current limitation is its based on bounding box center for proof of concept
- [ ] Multi-goal task planning
- [ ] VLM integration for open-vocabulary queries (extend current natural language input)

**Long-term:**
- [ ] New/custom city environment for true autonomous vehicle simulation (road, traffic lights, cars, signs, etc.)
- [ ] Visual SLAM fusion (ex: RTAB-Map + LiDAR)
- [ ] Multiple cameras for detection
- [ ] NeRF/Gaussian Splatting for photorealistic and custom simulation environments
- [ ] RL-based local planner

## Acknowledgements
- **Nav2** for ROS2 navigation framework
- **slam_toolbox**
- **m-explore-ros2** (robo-friends) for autonomous exploration
- **Ultralytics** for YOLOv10 
