# Important ROS Information
Below you will find information about all the ROS packages, nodes, topics used in this project.

## Packages & Nodes
Here is a list of packages. Underneath each package are nodes in that package.

------------------------------

### simulation
The major purpose of the simulation package is to connect our self-driving system to CARLA simulator. To run the package, please refer to the documentation [here](./src/simulation/README.md).

The simulation package can also run simulated camera inputs using the `camera_sim_node`

#### Nodes:
- `carla_client`
- `camera_sim_node`

#### Launch Files:
- `carla_client.launch`
- `carla_client_with_rviz.launch`
- `carla_client_with_rqt.launch`
- `start_camera_sim.launch`

------------------------------

### autopilot
The autopilot node is the brain of the self-driving car. It uses end-to-end
deep learning to predict the steering, acceleration and braking commands of 
the vehicle.  while subscribes to the camera feed. (Node currently functioning) The Arduino subsribes to the steering_cmds and controls the steering accordingly.

##### Nodes:
- `autopilot`
- `visualization`

##### Publishes
- `/vehicle/dbw/steering_cmds/`
- `/vehicle/dbw/cruise_cmds/`

##### Subscribes
- `/camera_node/image_raw`
- `/camera_node/image_sim`

------------------------------

### object_detection
YOLO (You Only Look Once) realtime object detection system.

#### Nodes:
-  `object_detection_node`

#### Publishes
- `/detection/object/detection_visualization/`
- `/detection/object/detection_result`

#### Subscribes:
- `/camera_node/image_raw`

------------------------------

### segmentation
Semantic segmentation node. Deep learning, ConvNets

##### Nodes:
- `segmentation_node`

##### Publishes
- `/segmentation/visualization/`
- `/segmentation/output`

##### Subscribes
- `/camera_node/image_raw`

------------------------------

### camera
The cameras are the main sensors of the self-driving car. 

##### Nodes:
- `camera_node`

##### Publishes
- `/camera_node/image_raw`

------------------------------

### driver
This is the main package of the project. It pulls together all the individual nodes to create a complete self-driving system.  

##### Nodes:
- `drive`

------------------------------

### gps
Used for localization. Currently using the Adafruit GPS module, serial communication.

##### Nodes:
- `gps_receiver`
- `nmea_topic_driver`
- `nmea_topic_serial_reader`

The GPS package manages and publishes the data received from a GPS module connected via serial. The package 

#### Publishes:

- `/sensor/gps/fix`
- `/sensor/gps/vel`

------------------------------

### data_logger
The logger node records and compiles a dataset for supervised machine learning purposes. The future is to use rosbag instead of the data_loggar node. (Node currently functioning)

##### Nodes:
- `logger`

------------------------------

### osm_cartography

#### Nodes:
- `osm_client`
- `osm_server`
- `viz_osm`

This package broadcasts and processes .osm files. OSM files are OpenStreetMap files which contain detailed information about the environment, 
such as coordinates of roads, building and landmarks. Currently, the main function of the package is to broadcast the osm info to rviz for 
visualization. (Node currently functioning)

### topics for visualization
- /visual/steering/angle_img
- /visual/detection/object/bbox_img
- /visual/detection/lane/marking_img
- /visual/segmentation/seg_img