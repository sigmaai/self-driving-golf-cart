execute_process(COMMAND "/home/neil/Workspace/self-driving-golf-cart/ros/src/cmake-build-debug/osm/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/neil/Workspace/self-driving-golf-cart/ros/src/cmake-build-debug/osm/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
