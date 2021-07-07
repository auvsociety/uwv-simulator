set(COMMON_VEC6_FILE_DIR ${PROJECT_SOURCE_DIR}/src/vec6/common)

include_directories(${COMMON_VEC6_FILE_DIR})

set(COMMON_VEC6_SIM_FILES   ${COMMON_VEC6_FILE_DIR}/terminal_getch.cpp
							${COMMON_VEC6_FILE_DIR}/vec6_comms.cpp
							${COMMON_VEC6_FILE_DIR}/vec6_controller.cpp
							${COMMON_VEC6_FILE_DIR}/vec6_simcontroller.cpp)

set(COMMON_VEC6_HARDWARE_FILES  ${COMMON_VEC6_FILE_DIR}/terminal_getch.cpp
								${COMMON_VEC6_FILE_DIR}/vec6_comms.cpp
								${COMMON_VEC6_FILE_DIR}/vec6_controller.cpp
								${COMMON_VEC6_FILE_DIR}/vec6_hardwarecontroller.cpp)

add_subdirectory(remote_control)
add_subdirectory(sauvc_tasks)