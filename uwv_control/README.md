# uwv_control

Contains files related to controlling the underwater vehicle. It is the main ROS package from which launch files related to testing and final implementation are executed.

## Usage
1. To simulate the **vec6** vehicle in the SAUVC swimming pool
	```bash
	roslaunch uwv_control sauvc_pool.launch
	```

2. To control the vehicle in ROV mode
	```bash
	rosrun uwv_control sim_remote_control
	```

3. To execute a sample set of simple tasks
	```bash
	rosrun uwv_control sim_sampletasks_exe
	```


<br/>

[Back to parent navigation](../README.md#navigate)