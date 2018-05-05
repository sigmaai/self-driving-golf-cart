execute_process(COMMAND "/home/yongyang/Workspace/self-driving-golf-cart/build/open_street_map/route_network/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/yongyang/Workspace/self-driving-golf-cart/build/open_street_map/route_network/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
