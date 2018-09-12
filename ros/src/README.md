# ROS Info
Below you will find all the packages, nodes, topics and other information about ROS in this project.

## Packages & Nodes
Here is a list of packages. Underneath each package are nodes in that package.

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

### gps

- gps_receiver
- nmea\_topic\_driver
- nmea\_topic\_serial_reader

The GPS package manages and publishes the data received from a GPS module connected via serial. The package publishes:

- /sensor/gps/fix
- /sensor/gps/vel

### steering_control

- steering_control

The steering control package controls the steering system of the vehicle. If publishes `/vehicle/dbw/steering_cmds/` while subscribes to the camera feed. (Node currently functioning) The Arduino subsribes to the steering_cmds and controls the steering accordingly.

### data_logger
- logger

The logger node records and compiles a dataset for supervised machine learning purposes. The future is to use rosbag instead of the data_loggar node. (Node currently functioning)

### cv_camera

- cv\_camera_node

This is the camera (perception sensor) package

### driver

- drive

The driver package is will run entire self-driving software. There are two launch files: `display.launch` & `drive.launch`. The display launch file runs rviz. The drive launch file will run the self-driving systems, i.e. steering control, cruise control, sensors... (drive.launch under development. display.launch functioning)

### osm_cartography
	
- osm_client
- osm_server
- viz_osm

This package broadcasts and processes .osm files. OSM files are OpenStreetMap files which contain detailed information about the environment, such as coordinates of roads, building and landmarks. Currently, the main function of the package is to broadcast the osm info to rviz for visualization. (Node currently functioning)

## Topics

### topics for the vehicle
- /cv\_camera\_node/image_raw
- /vehicle/dbw/steering_cmds/
- /vehicle/dbw/accel_cmds/
- /sensor/steering_encoder/value

### topics for visualization
- /visual/steering/angle_img
- /visual/detection/object/bbox_img
- /visual/detection/lane/marking_img
- /visual/segmentation/seg_img