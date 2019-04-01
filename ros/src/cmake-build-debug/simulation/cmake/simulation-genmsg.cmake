# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "simulation: 6 messages, 0 services")

set(MSG_I_FLAGS "-Isimulation:/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iderived_object_msgs:/opt/ros/melodic/share/derived_object_msgs/cmake/../msg;-Iackermann_msgs:/opt/ros/melodic/share/ackermann_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Ishape_msgs:/opt/ros/melodic/share/shape_msgs/cmake/../msg;-Iradar_msgs:/opt/ros/melodic/share/radar_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(simulation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg" NAME_WE)
add_custom_target(_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simulation" "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg" ""
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg" NAME_WE)
add_custom_target(_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simulation" "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg" NAME_WE)
add_custom_target(_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simulation" "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg" ""
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg" NAME_WE)
add_custom_target(_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simulation" "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg" ""
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg" NAME_WE)
add_custom_target(_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simulation" "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg" "simulation/EgoVehicleControlState:simulation/EgoVehicleControlCurrent:simulation/CarlaVehicleControl:simulation/EgoVehicleControlMaxima:std_msgs/Header:simulation/EgoVehicleControlTarget"
)

get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg" NAME_WE)
add_custom_target(_simulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simulation" "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation
)
_generate_msg_cpp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation
)
_generate_msg_cpp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation
)
_generate_msg_cpp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation
)
_generate_msg_cpp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation
)
_generate_msg_cpp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation
)

### Generating Services

### Generating Module File
_generate_module_cpp(simulation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(simulation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(simulation_generate_messages simulation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg" NAME_WE)
add_dependencies(simulation_generate_messages_cpp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg" NAME_WE)
add_dependencies(simulation_generate_messages_cpp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg" NAME_WE)
add_dependencies(simulation_generate_messages_cpp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg" NAME_WE)
add_dependencies(simulation_generate_messages_cpp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg" NAME_WE)
add_dependencies(simulation_generate_messages_cpp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg" NAME_WE)
add_dependencies(simulation_generate_messages_cpp _simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulation_gencpp)
add_dependencies(simulation_gencpp simulation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation
)
_generate_msg_eus(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation
)
_generate_msg_eus(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation
)
_generate_msg_eus(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation
)
_generate_msg_eus(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation
)
_generate_msg_eus(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation
)

### Generating Services

### Generating Module File
_generate_module_eus(simulation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(simulation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(simulation_generate_messages simulation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg" NAME_WE)
add_dependencies(simulation_generate_messages_eus _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg" NAME_WE)
add_dependencies(simulation_generate_messages_eus _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg" NAME_WE)
add_dependencies(simulation_generate_messages_eus _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg" NAME_WE)
add_dependencies(simulation_generate_messages_eus _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg" NAME_WE)
add_dependencies(simulation_generate_messages_eus _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg" NAME_WE)
add_dependencies(simulation_generate_messages_eus _simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulation_geneus)
add_dependencies(simulation_geneus simulation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation
)
_generate_msg_lisp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation
)
_generate_msg_lisp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation
)
_generate_msg_lisp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation
)
_generate_msg_lisp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation
)
_generate_msg_lisp(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation
)

### Generating Services

### Generating Module File
_generate_module_lisp(simulation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(simulation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(simulation_generate_messages simulation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg" NAME_WE)
add_dependencies(simulation_generate_messages_lisp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg" NAME_WE)
add_dependencies(simulation_generate_messages_lisp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg" NAME_WE)
add_dependencies(simulation_generate_messages_lisp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg" NAME_WE)
add_dependencies(simulation_generate_messages_lisp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg" NAME_WE)
add_dependencies(simulation_generate_messages_lisp _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg" NAME_WE)
add_dependencies(simulation_generate_messages_lisp _simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulation_genlisp)
add_dependencies(simulation_genlisp simulation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation
)
_generate_msg_nodejs(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation
)
_generate_msg_nodejs(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation
)
_generate_msg_nodejs(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation
)
_generate_msg_nodejs(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation
)
_generate_msg_nodejs(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(simulation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(simulation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(simulation_generate_messages simulation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg" NAME_WE)
add_dependencies(simulation_generate_messages_nodejs _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg" NAME_WE)
add_dependencies(simulation_generate_messages_nodejs _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg" NAME_WE)
add_dependencies(simulation_generate_messages_nodejs _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg" NAME_WE)
add_dependencies(simulation_generate_messages_nodejs _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg" NAME_WE)
add_dependencies(simulation_generate_messages_nodejs _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg" NAME_WE)
add_dependencies(simulation_generate_messages_nodejs _simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulation_gennodejs)
add_dependencies(simulation_gennodejs simulation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation
)
_generate_msg_py(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation
)
_generate_msg_py(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation
)
_generate_msg_py(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation
)
_generate_msg_py(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation
)
_generate_msg_py(simulation
  "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation
)

### Generating Services

### Generating Module File
_generate_module_py(simulation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(simulation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(simulation_generate_messages simulation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlState.msg" NAME_WE)
add_dependencies(simulation_generate_messages_py _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/CarlaVehicleControl.msg" NAME_WE)
add_dependencies(simulation_generate_messages_py _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlTarget.msg" NAME_WE)
add_dependencies(simulation_generate_messages_py _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlCurrent.msg" NAME_WE)
add_dependencies(simulation_generate_messages_py _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlInfo.msg" NAME_WE)
add_dependencies(simulation_generate_messages_py _simulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/neil/Workspace/self-driving-golf-cart/ros/src/simulation/msg/EgoVehicleControlMaxima.msg" NAME_WE)
add_dependencies(simulation_generate_messages_py _simulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simulation_genpy)
add_dependencies(simulation_genpy simulation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simulation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simulation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(simulation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET derived_object_msgs_generate_messages_cpp)
  add_dependencies(simulation_generate_messages_cpp derived_object_msgs_generate_messages_cpp)
endif()
if(TARGET ackermann_msgs_generate_messages_cpp)
  add_dependencies(simulation_generate_messages_cpp ackermann_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/simulation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(simulation_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET derived_object_msgs_generate_messages_eus)
  add_dependencies(simulation_generate_messages_eus derived_object_msgs_generate_messages_eus)
endif()
if(TARGET ackermann_msgs_generate_messages_eus)
  add_dependencies(simulation_generate_messages_eus ackermann_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/simulation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(simulation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET derived_object_msgs_generate_messages_lisp)
  add_dependencies(simulation_generate_messages_lisp derived_object_msgs_generate_messages_lisp)
endif()
if(TARGET ackermann_msgs_generate_messages_lisp)
  add_dependencies(simulation_generate_messages_lisp ackermann_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/simulation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(simulation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET derived_object_msgs_generate_messages_nodejs)
  add_dependencies(simulation_generate_messages_nodejs derived_object_msgs_generate_messages_nodejs)
endif()
if(TARGET ackermann_msgs_generate_messages_nodejs)
  add_dependencies(simulation_generate_messages_nodejs ackermann_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simulation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(simulation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET derived_object_msgs_generate_messages_py)
  add_dependencies(simulation_generate_messages_py derived_object_msgs_generate_messages_py)
endif()
if(TARGET ackermann_msgs_generate_messages_py)
  add_dependencies(simulation_generate_messages_py ackermann_msgs_generate_messages_py)
endif()
