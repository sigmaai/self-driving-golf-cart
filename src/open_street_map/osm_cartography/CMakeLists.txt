cmake_minimum_required(VERSION 2.8)
project(osm_cartography)

# list catkin dependencies
set(_ROS_PKG_DEPS
  dynamic_reconfigure
  geodesy
  geographic_msgs
  geometry_msgs
  rospy
  tf
  std_msgs
  visualization_msgs)


# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS ${_ROS_PKG_DEPS})
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_python_setup()

# auto-generate dynamic reconfiguration GUI: before catkin_package()
# and after catkin_python_setup().
generate_dynamic_reconfigure_options(cfg/VizOSM.cfg)

catkin_package(CATKIN_DEPENDS ${_ROS_PKG_DEPS})

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

install(DIRECTORY launch/ 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# unit tests
if (CATKIN_ENABLE_TESTING)

  catkin_add_nosetests(tests/test_geo_map.py)
  catkin_add_nosetests(tests/test_xml_map.py)

  # check all the launch files
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)

endif (CATKIN_ENABLE_TESTING)
