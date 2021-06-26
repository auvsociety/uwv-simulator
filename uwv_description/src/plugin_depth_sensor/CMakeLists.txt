############# Plugin for simulating depth sensor #############
add_library(plugin_depth SHARED
    plugin_depth.cpp
)

target_link_libraries(plugin_depth
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

set_target_properties(plugin_depth
		PROPERTIES
		LIBRARY_OUTPUT_DIRECTORY
		${PROJECT_SOURCE_DIR}/plugins
)