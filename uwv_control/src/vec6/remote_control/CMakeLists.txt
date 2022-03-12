add_executable(sim_remote_control   ${COMMON_SRC_FILES}
									${COMMON_VEC6_SIM_FILES}
									sim_remote_control.cpp)

target_link_libraries(sim_remote_control ${catkin_LIBRARIES})
add_dependencies(sim_remote_control uwv_control_generate_messages_cpp)

add_executable(hardware_remote_control   ${COMMON_SRC_FILES}
									${COMMON_VEC6_HARDWARE_FILES}
									hardware_remote_control.cpp)

target_link_libraries(hardware_remote_control ${catkin_LIBRARIES})
add_dependencies(hardware_remote_control uwv_control_generate_messages_cpp)
