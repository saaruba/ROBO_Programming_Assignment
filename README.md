# Emergency Inventory Robot - ROS 2 Coursework Project

## Project Overview

This project is an autonomous emergency equipment inventory robot developed using ROS 2 Humble, Gazebo Classic, Nav2, and OpenCV.

The aim of the project is to make a mobile robot automatically navigate inside a mapped indoor environment, detect emergency equipment, avoid duplicate counting, and generate a final inventory report.

The robot detects:

- Fire extinguisher - red object
- First aid kit - green object
- AED kit - blue object

The final output is saved into a structured `results.json` file with object counts and map-frame positions.

---

## Main Features

- Autonomous waypoint navigation using ROS 2 Nav2
- AMCL localisation using saved map
- Custom Gazebo simulation world
- RGB-D object detection
- HSV colour segmentation
- Contour and shape filtering
- Depth-based 3D projection
- TF transformation into map frame
- AMCL fallback if projection fails
- Grid-based and distance-based de-duplication
- Temporal sighting-lock to avoid repeated counting
- JSON inventory report generation

---
# Running the Full Project

Use **separate terminals** for each step.

---

## Terminal 1 — Launch Gazebo World

```bash
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py \
world:=src/emergency_inventory/emergency_inventory/worlds/final_world.world 
```

## Terminal 2 — Launch Nav2 + AMCL

```bash
ros2 launch limo_navigation limo_navigation.launch.py \
map:=src/emergency_inventory/emergency_inventory/maps/robo_map.yaml \
use_sim_time:=true
```

## Terminal 3 — Open RViz

```bash
rviz2
```
## Inside RViz:
Set Fixed Frame → map
Add:
/map → Map
/scan → LaserScan
Click 2D Pose Estimate and set robot start position

## Terminal 4 — Run Inventory Detection Node

```bash
ros2 run emergency_inventory inventory_node
```
## Terminal 5 — Run Waypoint Mission

```bash
ros2 run emergency_inventory waypoint_mission
```
## Check Final Results
```bash
cat /workspaces/cmp9767-ws-main/src/emergency_inventory/emergency_inventory/results.json
```
