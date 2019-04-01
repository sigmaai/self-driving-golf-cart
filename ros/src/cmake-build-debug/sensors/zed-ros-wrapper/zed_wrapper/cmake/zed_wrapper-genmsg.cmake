# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "zed_wrapper: 0 messages, 5 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(zed_wrapper_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv" NAME_WE)
add_custom_target(_zed_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "zed_wrapper" "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv" ""
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv" NAME_WE)
add_custom_target(_zed_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "zed_wrapper" "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv" ""
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv" NAME_WE)
add_custom_target(_zed_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "zed_wrapper" "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv" ""
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv" NAME_WE)
add_custom_target(_zed_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "zed_wrapper" "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv" ""
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv" NAME_WE)
add_custom_target(_zed_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "zed_wrapper" "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zed_wrapper
)
_generate_srv_cpp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zed_wrapper
)
_generate_srv_cpp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zed_wrapper
)
_generate_srv_cpp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zed_wrapper
)
_generate_srv_cpp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zed_wrapper
)

### Generating Module File
_generate_module_cpp(zed_wrapper
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zed_wrapper
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(zed_wrapper_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(zed_wrapper_generate_messages zed_wrapper_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_cpp _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_cpp _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_cpp _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_cpp _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_cpp _zed_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zed_wrapper_gencpp)
add_dependencies(zed_wrapper_gencpp zed_wrapper_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zed_wrapper_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zed_wrapper
)
_generate_srv_eus(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zed_wrapper
)
_generate_srv_eus(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zed_wrapper
)
_generate_srv_eus(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zed_wrapper
)
_generate_srv_eus(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zed_wrapper
)

### Generating Module File
_generate_module_eus(zed_wrapper
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zed_wrapper
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(zed_wrapper_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(zed_wrapper_generate_messages zed_wrapper_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_eus _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_eus _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_eus _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_eus _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_eus _zed_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zed_wrapper_geneus)
add_dependencies(zed_wrapper_geneus zed_wrapper_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zed_wrapper_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zed_wrapper
)
_generate_srv_lisp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zed_wrapper
)
_generate_srv_lisp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zed_wrapper
)
_generate_srv_lisp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zed_wrapper
)
_generate_srv_lisp(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zed_wrapper
)

### Generating Module File
_generate_module_lisp(zed_wrapper
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zed_wrapper
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(zed_wrapper_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(zed_wrapper_generate_messages zed_wrapper_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_lisp _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_lisp _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_lisp _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_lisp _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_lisp _zed_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zed_wrapper_genlisp)
add_dependencies(zed_wrapper_genlisp zed_wrapper_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zed_wrapper_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zed_wrapper
)
_generate_srv_nodejs(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zed_wrapper
)
_generate_srv_nodejs(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zed_wrapper
)
_generate_srv_nodejs(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zed_wrapper
)
_generate_srv_nodejs(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zed_wrapper
)

### Generating Module File
_generate_module_nodejs(zed_wrapper
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zed_wrapper
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(zed_wrapper_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(zed_wrapper_generate_messages zed_wrapper_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_nodejs _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_nodejs _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_nodejs _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_nodejs _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_nodejs _zed_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zed_wrapper_gennodejs)
add_dependencies(zed_wrapper_gennodejs zed_wrapper_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zed_wrapper_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper
)
_generate_srv_py(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper
)
_generate_srv_py(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper
)
_generate_srv_py(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper
)
_generate_srv_py(zed_wrapper
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper
)

### Generating Module File
_generate_module_py(zed_wrapper
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(zed_wrapper_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(zed_wrapper_generate_messages zed_wrapper_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/set_initial_pose.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_py _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_odometry.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_py _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/reset_tracking.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_py _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/stop_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_py _zed_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/sensors/zed-ros-wrapper/zed_wrapper/srv/start_svo_recording.srv" NAME_WE)
add_dependencies(zed_wrapper_generate_messages_py _zed_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zed_wrapper_genpy)
add_dependencies(zed_wrapper_genpy zed_wrapper_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zed_wrapper_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zed_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zed_wrapper
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zed_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zed_wrapper
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zed_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zed_wrapper
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zed_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zed_wrapper
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zed_wrapper
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
