Emergency Inventory Robot – CMP9767

Autonomous Navigation and Vision-Based Object Inventory

1. Project Overview

This project implements an autonomous mobile robot system using ROS 2 Humble that can:

Navigate autonomously inside a custom Gazebo world

Localise itself using AMCL

Follow a predefined inspection route using waypoints

Detect emergency-related objects using camera-based vision

Identify objects based on both colour and shape

Count detected objects without double-counting

Safely return to the starting position after inspection

The system integrates navigation, localisation, mapping, and computer vision, following the methods taught in the CMP9767 workshops.

2. System Requirements

Ubuntu 22.04

ROS 2 Humble

Gazebo (Classic)

Nav2 Stack

slam_toolbox

OpenCV (via cv_bridge)

This project was developed and tested inside the provided Docker development environment.

3. Project Structure
emergency_inventory/
├── emergency_inventory/
│   ├── inventory_detector.py
│   ├── waypoint_mission.py
│   └── __init__.py
├── worlds/
│   └── robo_programming.world
├── maps/
│   ├── robo_map.yaml
│   └── robo_map.pgm
├── setup.py
├── package.xml
└── README.md

4. Building the Workspace

After extracting the ZIP file:

cd cmp9767-ws-main
colcon build --symlink-install
source install/setup.bash


Verify the executables:

ros2 pkg executables emergency_inventory


You should see:

waypoint_mission

inventory_detector

5. Launching the Simulation
Step 1 – Start Gazebo with the custom world
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py \
world:=src/emergency_inventory/emergency_inventory/worlds/robo_programming.world

Step 2 – Start Navigation with the saved map
ros2 launch limo_navigation limo_navigation.launch.py \
map:=src/emergency_inventory/emergency_inventory/maps/robo_map.yaml \
use_sim_time:=true

Step 3 – Open RViz (Navigation View)
rviz2 -d /opt/ros/lcas/install/limo_navigation/share/limo_navigation/rviz/limo_navigation.rviz


In RViz:

Ensure Map, TF, RobotModel, LaserScan, and AMCL cloud are enabled

If required, initialise the robot pose using 2D Pose Estimate

6. Running the Inventory node

The detector subscribes to the robot camera feed and performs:

Colour segmentation

Shape analysis

Object classification

Duplicate suppression

Object counting

Run the detector node:

ros2 run emergency_inventory inventory_node


Detected objects and running counts are printed to the terminal.

7. Running the Autonomous Inspection Mission

The waypoint mission uses Nav2 Simple Commander to:

Follow all predefined inspection waypoints

Avoid obstacles dynamically

Replan paths if blocked

Return to the starting location at the end

Run:

ros2 run emergency_inventory waypoint_mission


Mission progress and waypoint status are displayed in the terminal.

8. Object Detection Logic

Objects are identified using a combination of colour and shape:

Object Type	Colour	Shape
Fire Extinguisher	Red	Vertical cylinder
Medical Box	Green	Rectangular box
Tool Case	Blue	Rectangular box

To avoid double-counting:

Each detected object is stored in map coordinates

New detections are ignored if they are within a threshold distance of a previously detected object

9. Navigation Behaviour

The robot uses Nav2 global and local planners

If a planned path becomes blocked, the robot automatically replans

AMCL continuously refines localisation using LiDAR and the saved map

If localisation quality degrades, AMCL can be reinitialised using:

ros2 service call /reinitialize_global_localization std_srvs/srv/Empty

10. Limitations and Future Improvements

Object detection is rule-based and sensitive to lighting conditions

Detection accuracy could be improved using CNN-based detectors (e.g. YOLO)

Dynamic object tracking and semantic mapping could enhance performance

11. Conclusion

This project demonstrates a complete autonomous robotic inspection system, integrating:

Navigation

Localisation

Mapping

Vision-based object detection

Decision-making and task execution


Project GIT Repositry :

https://github.com/saaruba/ROBO_Programming_Assignment.git