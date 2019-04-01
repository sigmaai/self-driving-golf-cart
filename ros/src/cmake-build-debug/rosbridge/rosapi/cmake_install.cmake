# Install script for directory: /home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/neil/Workspace/self-driving-golf-cart/ros/src/cmake-build-debug/rosbridge/rosapi/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/msg" TYPE FILE FILES "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/msg/TypeDef.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/srv" TYPE FILE FILES
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/DeleteParam.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/GetActionServers.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/GetParam.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/GetParamNames.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/GetTime.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/HasParam.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/MessageDetails.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/Nodes.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/NodeDetails.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/Publishers.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/SearchParam.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/ServiceHost.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/ServiceNode.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/ServiceProviders.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/ServiceRequestDetails.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/ServiceResponseDetails.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/Services.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/ServicesForType.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/ServiceType.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/SetParam.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/Subscribers.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/Topics.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/TopicsForType.srv"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/srv/TopicType.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/cmake" TYPE FILE FILES "/home/neil/Workspace/self-driving-golf-cart/ros/src/cmake-build-debug/rosbridge/rosapi/catkin_generated/installspace/rosapi-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/neil/Workspace/self-driving-golf-cart/ros/devel/include/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/neil/Workspace/self-driving-golf-cart/ros/devel/share/roseus/ros/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/neil/Workspace/self-driving-golf-cart/ros/devel/share/common-lisp/ros/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/neil/Workspace/self-driving-golf-cart/ros/devel/share/gennodejs/ros/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/neil/Workspace/self-driving-golf-cart/ros/devel/lib/python2.7/dist-packages/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/neil/Workspace/self-driving-golf-cart/ros/devel/lib/python2.7/dist-packages/rosapi" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/neil/Workspace/self-driving-golf-cart/ros/devel/lib/python2.7/dist-packages/rosapi" FILES_MATCHING REGEX "/home/neil/Workspace/self-driving-golf-cart/ros/devel/lib/python2.7/dist-packages/rosapi/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/neil/Workspace/self-driving-golf-cart/ros/src/cmake-build-debug/rosbridge/rosapi/catkin_generated/installspace/rosapi.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/cmake" TYPE FILE FILES "/home/neil/Workspace/self-driving-golf-cart/ros/src/cmake-build-debug/rosbridge/rosapi/catkin_generated/installspace/rosapi-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/cmake" TYPE FILE FILES
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/cmake-build-debug/rosbridge/rosapi/catkin_generated/installspace/rosapiConfig.cmake"
    "/home/neil/Workspace/self-driving-golf-cart/ros/src/cmake-build-debug/rosbridge/rosapi/catkin_generated/installspace/rosapiConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi" TYPE FILE FILES "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rosapi" TYPE PROGRAM FILES "/home/neil/Workspace/self-driving-golf-cart/ros/src/rosbridge/rosapi/scripts/rosapi_node")
endif()

