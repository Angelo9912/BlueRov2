# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tesi_bluerov2: 4 messages, 0 services")

set(MSG_I_FLAGS "-Itesi_bluerov2:/home/angelo/catkin_ws/src/tesi_bluerov2/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tesi_bluerov2_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg" NAME_WE)
add_custom_target(_tesi_bluerov2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tesi_bluerov2" "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg" ""
)

get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg" NAME_WE)
add_custom_target(_tesi_bluerov2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tesi_bluerov2" "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg" ""
)

get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg" NAME_WE)
add_custom_target(_tesi_bluerov2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tesi_bluerov2" "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg" ""
)

get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg" NAME_WE)
add_custom_target(_tesi_bluerov2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tesi_bluerov2" "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_cpp(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_cpp(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_cpp(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesi_bluerov2
)

### Generating Services

### Generating Module File
_generate_module_cpp(tesi_bluerov2
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesi_bluerov2
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tesi_bluerov2_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tesi_bluerov2_generate_messages tesi_bluerov2_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_cpp _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_cpp _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_cpp _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_cpp _tesi_bluerov2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesi_bluerov2_gencpp)
add_dependencies(tesi_bluerov2_gencpp tesi_bluerov2_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesi_bluerov2_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_eus(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_eus(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_eus(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesi_bluerov2
)

### Generating Services

### Generating Module File
_generate_module_eus(tesi_bluerov2
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesi_bluerov2
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tesi_bluerov2_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tesi_bluerov2_generate_messages tesi_bluerov2_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_eus _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_eus _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_eus _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_eus _tesi_bluerov2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesi_bluerov2_geneus)
add_dependencies(tesi_bluerov2_geneus tesi_bluerov2_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesi_bluerov2_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_lisp(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_lisp(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_lisp(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesi_bluerov2
)

### Generating Services

### Generating Module File
_generate_module_lisp(tesi_bluerov2
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesi_bluerov2
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tesi_bluerov2_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tesi_bluerov2_generate_messages tesi_bluerov2_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_lisp _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_lisp _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_lisp _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_lisp _tesi_bluerov2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesi_bluerov2_genlisp)
add_dependencies(tesi_bluerov2_genlisp tesi_bluerov2_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesi_bluerov2_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_nodejs(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_nodejs(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_nodejs(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesi_bluerov2
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tesi_bluerov2
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesi_bluerov2
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tesi_bluerov2_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tesi_bluerov2_generate_messages tesi_bluerov2_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_nodejs _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_nodejs _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_nodejs _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_nodejs _tesi_bluerov2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesi_bluerov2_gennodejs)
add_dependencies(tesi_bluerov2_gennodejs tesi_bluerov2_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesi_bluerov2_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_py(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_py(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesi_bluerov2
)
_generate_msg_py(tesi_bluerov2
  "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesi_bluerov2
)

### Generating Services

### Generating Module File
_generate_module_py(tesi_bluerov2
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesi_bluerov2
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tesi_bluerov2_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tesi_bluerov2_generate_messages tesi_bluerov2_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_py _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/Floats_String.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_py _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/waypoints.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_py _tesi_bluerov2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/angelo/catkin_ws/src/tesi_bluerov2/msg/buoy.msg" NAME_WE)
add_dependencies(tesi_bluerov2_generate_messages_py _tesi_bluerov2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tesi_bluerov2_genpy)
add_dependencies(tesi_bluerov2_genpy tesi_bluerov2_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tesi_bluerov2_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesi_bluerov2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tesi_bluerov2
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tesi_bluerov2_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesi_bluerov2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tesi_bluerov2
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tesi_bluerov2_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesi_bluerov2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tesi_bluerov2
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tesi_bluerov2_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesi_bluerov2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tesi_bluerov2
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tesi_bluerov2_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesi_bluerov2)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesi_bluerov2\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tesi_bluerov2
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tesi_bluerov2_generate_messages_py std_msgs_generate_messages_py)
endif()
