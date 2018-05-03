cmake_minimum_required(VERSION 2.8)
project(route_network)

# list catkin dependencies
set(_ROS_PKG_DEPS
    dynamic_reconfigure
    geodesy
    geographic_msgs
    geometry_msgs
    nav_msgs
    rospy
    visualization_msgs)


# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS ${_ROS_PKG_DEPS})
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_python_setup()

# auto-generate dynamic reconfiguration GUI: before catkin_package()
# and after catkin_python_setup().
generate_dynamic_reconfigure_options(cfg/RouteNetwork.cfg)

catkin_package(CATKIN_DEPENDS ${_ROS_PKG_DEPS})

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

install(DIRECTORY launch/ 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# unit tests
if (CATKIN_ENABLE_TESTING)
  
  find_package(catkin REQUIRED rostest roslaunch)

  add_rostest(tests/planner.test)

  roslaunch_add_file_check(launch)

endif (CATKIN_ENABLE_TESTING)
