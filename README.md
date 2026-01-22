# Single Robot Navigation - Hardware Testing

## 1. Overview

This module implements an autonomous navigation and exploration system for the AgileX LIMO PRO robot using ROS Noetic. The system enables the robot to autonomously explore unknown environments, construct maps using SLAM, localize itself in real time, and detect visual targets using an onboard camera.

The implementation was validated on real hardware for single-robot navigation and extended to simulation for multi-robot exploration and map merging.

**Hardware:**
-AgileX LIMO PRO
- 2D LiDAR
- RGB camera

## 2. Mapping

Uses **SLAM Toolbox** for real-time mapping during autonomous exploration. **Explore Lite** identifies unexplored areas and navigates to them automatically.

- LiDAR scans environment continuously
- SLAM builds 2D occupancy grid map
- Loop closure corrects drift
- Hardware testing generated accurate real-world maps

## 3. Localization

**SLAM-based localization** tracks robot position while building the map.

- Scan matching matches LiDAR data to map
- Position updates at >10Hz
- Pose graph optimization reduces errors
- Position accuracy < 0.1m

## 4. Hardware Testing & Results

- ✅ Autonomous exploration and mapping
- ✅ Accurate localization 
- ✅ Complete environment coverage

<img width="433" height="468" alt="image" src="https://github.com/user-attachments/assets/e297e69d-555d-40fe-9f57-e2d54f020e28" />

![alt text](<Video Project-1.gif>)

## 5. Multi-Robot Simulation

A simulation-only setup was implemented to evaluate multi-robot exploration and map merging using three LIMO robots in the same environment.

- Simulation-only setup with three LIMO robots exploring the same world
- Launch: `roslaunch robotics_project_pkg limo_three_robot.launch`
- Each robot runs its own SLAM and exploration; maps merged by `map_merger.py` into `/map_merged`
- Use RViz config `rviz/multi_config.rviz` to monitor all robots and merged map
- Map merge: resolution from first map, frame `world`, overlapping cells favor obstacles

**Visualization**
- RViz configuration: rviz/multi_config.rviz
- Displays individual robot maps, trajectories, and the merged global map

**Map merging strategy**
- Map resolution inherited from the first received map
- Global frame: world
- Overlapping cells prioritize occupied (obstacle) values

  ![alt text](gazebo_map.png) ![alt text](map_exploration.png)

## 6. Target Detection

The vision module performs **visual target search** using the robot’s RGB camera during autonomous exploration.

### Implementation

- Vision node: `target_detector.py`
- Detects **red box targets** using HSV color thresholding
- Designed for **real-time operation** while the robot is navigating and mapping

### Published Topics

- `/target_detected` – Boolean flag indicating whether the target has been detected
- `/detection_status` – Textual status message describing the detection state
- `/debug_image` – Visualization image showing detection results
