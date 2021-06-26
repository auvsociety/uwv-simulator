# uwv_description/src/plugin_depth_sensor

- A depth sensor plugin that publishes to a ROS topic
- To use the plugin, add the following snippet to the URDF file:
	```xml
	<gazebo>
		<plugin name='<some-unique-name-for-plugin>' filename='libplugin_depth.so'>
			<bodyName> name-of-link-used-as-depth-sensor </bodyName>
			<depthTopicName> ros-topic-to-publish-to </depthTopicName>
			<noise> noise-value-between-0-and-1 </noise>
		</plugin>
	</gazebo>
	```

	Example use-case in `vec6.xacro`
	```xml
	<gazebo>
	  <plugin name='depth_plugin' filename='libplugin_depth.so'>
		<bodyName>depth_sensor</bodyName>
		<depthTopicName>/uwv/vec6/sim_depth</depthTopicName>
		<noise>0.01</noise>
		</plugin>
  	</gazebo>
	```