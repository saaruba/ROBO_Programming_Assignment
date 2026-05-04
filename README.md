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

For gettting waypoints

ros2 topic echo /amcl_pose

Shore Exploration WayPoints Quadinates

[1.1554815942458223, 1.785122569857626, 0.0, 0.0, 0.6977797928649597, 0.7163123345785231],
[-0.5631636842152836, 3.9325931549461832, 0.0, 0.0, 0.9968489209909983, 0.07932356975755912],
[-3.768330664455778,  1.585779685398086, 0.0, 0.0, -0.796618344327087, 0.6044825998170423],
[-4.500376213834338, -1.619989926012594, 0.0, 0.0, -0.988462228360209, 0.15146756453171759],
[-6.974178425972754, -0.34232484469376173, 0.0, 0.0, 0.30038468844336963, 0.9538181372509016],
[-4.186469607954447, 2.618870332688237, 0.0, 0.0, 0.4146507887215101, 0.9099806170532587],
[-1.3671068250153218, 3.5694767952537623, 0.0, 0.0, -0.5426557839227355,  0.8399551774798472],
[1.5721114481691596, 1.014720590540959, 0.0, 0.0, -0.6617280339215736,  0.7497439623780165],
[0.7754187991190852, -0.11233475504065969, 0.0, 0.0, -0.15879910650969623,  0.9873109154525347],

Full map Exploration WayPoints Quadinates

              [0.4380440960076984, -0.30017426665114455, 0.0, 0.0, 0.3180586709926065, 0.9480710320468698],
              [1.2917316552750913, 1.3395856515052869, 0.0, 0.0, 0.5593811599339845, 0.8289105608634203],
              [1.5137642299622605, 3.7317498348382148, 0.0, 0.0, 0.8245023571578856, 0.5658585185725231],
              [-1.4596853809263386, 4.351749368545111, 0.0, 0.0, -0.9495298982077545, 0.3136765410571396],
              [-3.511686499464037, 3.6835452723832023, 0.0, 0.0, -0.891254421912406, 0.4535036443310936],
              [-3.7600257744766648, 0.026575861573809072, 0.0, 0.0, -0.7181551417321504, 0.6958830306909882],
              [-4.395047713571779, -2.3263015888048595, 0.0, 0.0, -0.9915982884981513, 0.12935545696852882],
              [-7.449072207369064, -2.8823181476311213, 0.0, 0.0, 0.8219698017509366, 0.5695310746654005],
              [-7.906326019694105, -0.9245097417426785, 0.0, 0.0, 0.6415269502258909, 0.7671004967628865],
              [-7.026622723663798, 0.34973345830635066, 0.0, 0.0, 0.23995693446932348, 0.9707835338529824],
              [-3.912056464922836, 1.6556356976778395, 0.0, 0.0, 0.4207351905551278, 0.9071834982122086],
              [-2.027907414118563, 4.12780583771418, 0.0, 0.0, -0.48735511374436047, 0.8732038668645608],
              [-1.2221214449184898, -3.3551669394422463, 0.0, 0.0, -0.8819189081205386, 0.9975891122311885],
              [1.704894046754486, -3.363856041126271, 0.0, 0.0, -0.06939714084736646, -0.8819189081205386],
              [-1.5583339546094246, -5.244009840714862, 0.0, 0.0, -0.9742772592720974, 0.2253526615446337],
              [-4.345883403876832, -4.788514179417724, 0.0, 0.0, 0.1858277962438677, 0.9825823274123892],
              [-1.569044845369205, -7.161639222881427, 0.0, 0.0, -0.9866277838434035, 0.1629896197561493],
              [-6.832440742091754, -6.939090311729447, 0.0, 0.0, 0.875122754759805, 0.4839009858448421],
              [-6.6257464391661935, -4.771148787468491, 0.0, 0.0, -0.4849353876442543, 0.8745499813105692],
              [-1.3330199215404106, -7.288032461954166, 0.0, 0.0, 0.10677527324132809, 0.9942831794937697],
              [5.4406620484181705, -6.055436624779557, 0.0, 0.0, 0.9050011303013623, 0.42540916087133873],
              [5.344888358338532, -2.385667227254904, 0.0, 0.0, 0.30021416140464807, 0.9538718243517332],
              [5.031826573347964, 3.478023304748386, 0.0, 0.0, 0.29311597804087214, 0.9560768920003991],
              [8.961607670217626, 4.792547380662433, 0.0, 0.0, -0.5654237402385451, 0.8248005783064499],
              [7.500587747689686, 0.6642534281526508, 0.0, 0.0, -0.6130819825765111, 0.7900192925745892],
              [7.910669990913888, -8.000716869076049, 0.0, 0.0, 0.8584421660848659, 0.5129103698381653],
              [5.180036198503661, 2.761183283110034, 0.0, 0.0, -0.8907959733836289, 0.45440349228798077],
              [6.071865837038928, -5.8038085191674735, 0.0, 0.0, -0.8823300656069174, 0.47063112447658284],
              [0.6231136396647378, -7.572336410229179, 0.0, 0.0, 0.9355388338293172, 0.35322385309783544],
              [-1.8457560138700526, -1.2581382408655117, 0.0, 0.0, 0.7227624185803786, 0.6910965824599639],
              [-0.22022726168401346, 3.0054245006567957, 0.0, 0.0, 0.5407743433633871, 0.8411677059658779],
              [1.516335030113087, 1.2285070618712433, 0.0, 0.0, -0.8958543819991882, 0.44434775373895236],
              [0.16668358796585533, 0.13621706608333062, 0.0, 0.0, -0.1711037268864201, 0.9852530206224072]