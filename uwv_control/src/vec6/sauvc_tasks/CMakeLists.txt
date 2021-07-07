set(TASK_DEF_SRC_FILES tasks.cpp
					   sim_task_list.cpp)

add_executable(sim_sampletasks_exe  ${COMMON_SRC_FILES}
									${COMMON_VEC6_SIM_FILES}
									${TASK_DEF_SRC_FILES}
									sim_sampletasks_exe.cpp)

target_link_libraries(sim_sampletasks_exe ${catkin_LIBRARIES})