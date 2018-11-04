# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(cv_camera_CONFIG_INCLUDED)
  return()
endif()
set(cv_camera_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(cv_camera_SOURCE_PREFIX /home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/cv_camera)
  set(cv_camera_DEVEL_PREFIX /home/neil/Workspace/self-driving-golf-cart/ros/devel)
  set(cv_camera_INSTALL_PREFIX "")
  set(cv_camera_PREFIX ${cv_camera_DEVEL_PREFIX})
else()
  set(cv_camera_SOURCE_PREFIX "")
  set(cv_camera_DEVEL_PREFIX "")
  set(cv_camera_INSTALL_PREFIX /home/neil/Workspace/self-driving-golf-cart/ros/install)
  set(cv_camera_PREFIX ${cv_camera_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'cv_camera' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(cv_camera_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/cv_camera/include;/usr/local/include;/usr/local/include/opencv " STREQUAL " ")
  set(cv_camera_INCLUDE_DIRS "")
  set(_include_dirs "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/cv_camera/include;/usr/local/include;/usr/local/include/opencv")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT "http://wiki.ros.org/cv_camera " STREQUAL " ")
    set(_report "Check the website 'http://wiki.ros.org/cv_camera' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Takashi Ogura <t.ogura@gmail.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${cv_camera_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'cv_camera' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'cv_camera' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/cv_camera/${idir}'.  ${_report}")
    endif()
    _list_append_unique(cv_camera_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "cv_camera;/usr/local/lib/libopencv_cudaimgproc.so.3.4.1;/usr/local/lib/libopencv_video.so.3.4.1;/usr/local/lib/libopencv_cudacodec.so.3.4.1;/usr/local/lib/libopencv_flann.so.3.4.1;/usr/local/lib/libopencv_imgproc.so.3.4.1;/usr/local/lib/libopencv_core.so.3.4.1;/usr/local/lib/libopencv_cudafilters.so.3.4.1;/usr/local/lib/libopencv_cudabgsegm.so.3.4.1;/usr/local/lib/libopencv_stitching.so.3.4.1;/usr/local/lib/libopencv_cudalegacy.so.3.4.1;/usr/local/lib/libopencv_photo.so.3.4.1;/usr/local/lib/libopencv_shape.so.3.4.1;/usr/local/lib/libopencv_cudev.so.3.4.1;/usr/local/lib/libopencv_cudawarping.so.3.4.1;/usr/local/lib/libopencv_imgcodecs.so.3.4.1;/usr/local/lib/libopencv_dnn.so.3.4.1;/usr/local/lib/libopencv_viz.so.3.4.1;/usr/local/lib/libopencv_ml.so.3.4.1;/usr/local/lib/libopencv_cudaobjdetect.so.3.4.1;/usr/local/lib/libopencv_videoio.so.3.4.1;/usr/local/lib/libopencv_cudafeatures2d.so.3.4.1;/usr/local/lib/libopencv_videostab.so.3.4.1;/usr/local/lib/libopencv_highgui.so.3.4.1;/usr/local/lib/libopencv_calib3d.so.3.4.1;/usr/local/lib/libopencv_features2d.so.3.4.1;/usr/local/lib/libopencv_cudaarithm.so.3.4.1;/usr/local/lib/libopencv_cudastereo.so.3.4.1;/usr/local/lib/libopencv_superres.so.3.4.1;/usr/local/lib/libopencv_objdetect.so.3.4.1;/usr/local/lib/libopencv_cudaoptflow.so.3.4.1")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND cv_camera_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND cv_camera_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND cv_camera_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/neil/Workspace/self-driving-golf-cart/ros/devel/lib;/opt/ros/melodic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(cv_camera_LIBRARY_DIRS ${lib_path})
      list(APPEND cv_camera_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'cv_camera'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND cv_camera_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(cv_camera_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${cv_camera_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "image_transport;roscpp;cv_bridge;sensor_msgs;nodelet;camera_info_manager")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 cv_camera_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${cv_camera_dep}_FOUND)
      find_package(${cv_camera_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${cv_camera_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(cv_camera_INCLUDE_DIRS ${${cv_camera_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(cv_camera_LIBRARIES ${cv_camera_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${cv_camera_dep}_LIBRARIES})
  _list_append_deduplicate(cv_camera_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(cv_camera_LIBRARIES ${cv_camera_LIBRARIES})

  _list_append_unique(cv_camera_LIBRARY_DIRS ${${cv_camera_dep}_LIBRARY_DIRS})
  list(APPEND cv_camera_EXPORTED_TARGETS ${${cv_camera_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${cv_camera_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
