# Install script for directory: /home/yongyang/Workspace/self-driving-golf-cart/src/open_street_map/osm_cartography

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yongyang/Workspace/self-driving-golf-cart/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  include("/home/yongyang/Workspace/self-driving-golf-cart/build/open_street_map/osm_cartography/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osm_cartography" TYPE FILE FILES "/home/yongyang/Workspace/self-driving-golf-cart/devel/include/osm_cartography/VizOSMConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/yongyang/Workspace/self-driving-golf-cart/devel/lib/python2.7/dist-packages/osm_cartography/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/osm_cartography" TYPE DIRECTORY FILES "/home/yongyang/Workspace/self-driving-golf-cart/devel/lib/python2.7/dist-packages/osm_cartography/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yongyang/Workspace/self-driving-golf-cart/build/open_street_map/osm_cartography/catkin_generated/installspace/osm_cartography.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/osm_cartography/cmake" TYPE FILE FILES
    "/home/yongyang/Workspace/self-driving-golf-cart/build/open_street_map/osm_cartography/catkin_generated/installspace/osm_cartographyConfig.cmake"
    "/home/yongyang/Workspace/self-driving-golf-cart/build/open_street_map/osm_cartography/catkin_generated/installspace/osm_cartographyConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/osm_cartography" TYPE FILE FILES "/home/yongyang/Workspace/self-driving-golf-cart/src/open_street_map/osm_cartography/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/osm_cartography" TYPE DIRECTORY FILES "/home/yongyang/Workspace/self-driving-golf-cart/src/open_street_map/osm_cartography/scripts/" USE_SOURCE_PERMISSIONS)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/osm_cartography/launch" TYPE DIRECTORY FILES "/home/yongyang/Workspace/self-driving-golf-cart/src/open_street_map/osm_cartography/launch/")
endif()

