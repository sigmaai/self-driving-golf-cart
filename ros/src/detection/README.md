# Welcome to Detection

This is the detection package. There are three nodes.

1. object_detector
2. lane_detector
3. tl_node

## Topics:
Publishes:
/visualization/detection/object_detection
/visualization/detection/lane_detection
/visualization/detection/tl

For visualization purposes

## Launch files

detection_master.launch
lane_detection.launch
object_detection.launch
tl_detection.launch

Note, detection_master is meant to be run by the driver node
The other launch files can be run individually, without starting the driver. The other launch files runs visualization by default