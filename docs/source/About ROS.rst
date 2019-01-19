About ROS
=========

====================
ROS Packages & Nodes
====================

The self-driving golf cart run on the robotics middleware called ROS (Robot Operating System). Below you will find all the packages, nodes, topics and other information about ROS in this project. If you are new to ROS, I highly recommend you to check out their tutorials and examples on their website: http://wiki.ros.org/ROS/Tutorials

gps
---
The GPS package functions as a rudimentary localization sensor. Below are the nodes in the package. 

node(s):

- gps_receiver
- nmea_topic_driver
- nmea_topic_serial_reader

The GPS package manages and publishes the data received from a GPS module connected via serial. The package publishes:

topic(s)

- /sensor/gps/fix
- /sensor/gps/vel

steering_control
----------------
- steering_control

The steering control package controls the steering system of the vehicle. If publishes $ /vehicle/dbw/steering_cmds/ while subscribes to the camera feed. (Node currently functioning) The Arduino subsribes to the steering_cmds and controls the steering accordingly.

cv_camera
---------
This is the camera (perception sensor) package. 

- cv_camera_node

driver
------
The driver package is will run entire self-driving software. There are two launch files: `display.launch` & `drive.launch`. The display launch file runs rviz. The drive launch file will run the self-driving systems, i.e. steering control, cruise control, sensors... (drive.launch under development. display.launch functioning)

node(s):

- drive

launch files:

- drive.launch
- display.launch

osm_cartography
---------------
This package broadcasts and processes .osm files. OSM files are OpenStreetMap files which contain detailed information about the environment, such as coordinates of roads, building and landmarks. Currently, the main function of the package is to broadcast the osm info to rviz for visualization. (Node currently functioning)

node(s): 

- osm_client
- osm_server
- viz_osm

### localization

### path_planning

### cruise_control

- cruise_controller

### tl_detection

- tl_detection

### lane_detection
	
- lane_detection

### object_detection

- object_detection

### segmentation

- segmentation

====================
Important ROS Topics
====================

topics for the DBW (Drive by Wire) system
-----------------------------------------
- /cv\_camera\_node/image_raw
- /vehicle/dbw/steering_cmds/
- /vehicle/dbw/cruise_cmds/
- /sensor/steering_encoder/value

topics for visualization
------------------------
- /visual/steering/angle_img
- /visual/detection/object/bbox_img (x)
- /visual/detection/lane/marking_img (x)
- /visual/segmentation/seg_img (x)

(x) implementing in progress