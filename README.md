# uwv-simulator

## Citation
Use the following snippet to cite our work:

M. N. Mehta, S. Mylraj and V. N. Bhate, "Development of AUV for SAUVC During COVID-19," OCEANS 2021: San Diego – Porto, 2021, pp. 1-8, doi: 10.23919/OCEANS44145.2021.9705822.

## Repository Overview
The **uwv-simulator** is an underwater vehicle (UWV) simulator developed by the [AUV Society], IIITDM Kancheepuram, India. This repository contains a compilation of ROS packages that aid in the development of an underwater vehicle. This simulation test-bed can used to implement, test, and get feedback on the control and vision algorithms for an underwater vehicle. 

> The team had intially released a raw version of their work through the [sauvc-simulations] public repository. Due to presence of legacy ROS packages in the previous repository, this new repository is created with more clean code and better documentation.

### Key features
-  Tested on ROS-Noetic + Gazebo-11
- Swimming pool environment of the Singapore Autonomous Underwater Challenge (SAUVC) has been built with props that can be placed dynamically at the beginning of simulation
- An underwater vehicle (named **vec6**) with six thrusters, amongst which four are in a vectored configuration is included
- A simple and efficient vectored thruster algorithm is developed enabling the vehicle to surge, sway, and yaw simultaneously
- The software stack is designed to execute the top-level algorithms both in the simulation and the hardware, with minor changes at low-level methods. The motivation for adopting this approach was to develop a framework that anyone willing to work on higher level problems like motion planning, etc. can easily do so without the need to worry about the lower-level logic.

### Packages
1. [uwv_control] : Contains files related to controlling the underwater vehicle. It is the main ROS package from which launch files related to testing and final implementation are executed.
2. [uwv_description] : Contains files and models describing the underwater vehicle and the sensors attached to it.
3. [uwv_env] : Contains files and models describing the environment.

### Installation
To make use of the repository and the components present in it, follow the subsequent steps:

1. Create a new `catkin_workspace` named `uwv_ws` (Recommended)

2. Clone the [freefloating-gazebo] ROS package to `uwv_ws/src`:

	```bash
	git clone https://github.com/freefloating-gazebo/freefloating_gazebo.git
	```

	`catkin_make` the workspace and add `uwv_ws/devel/setup.bash` to `~/.bashrc`.

3. Clone this repository to `uwv_ws/src`:

	```bash
	git clone https://github.com/auvsocietyiiitdm/uwv-simulator.git
	```

4. Follow the instructions presented [here](https://github.com/auvsocietyiiitdm/darknet_ros.git) to install `darknet_ros` for `uwv-simulator`.

5. (optional) Obtain our utility tool, [uwv-launcher] to create the SAUVC pool environment.

### Navigate
1. [uwv_control](./uwv_control)
2. [uwv_description](./uwv_description)
3. [uwv_env](./uwv_env)
4. [On uwv-simulator namespaces](./docs/uwv-ns.md)

## History
SAUVC 2020 was the AUV Society of IIITDM Kancheepuram’s second attempt for the challenge. The team developed an underwater equipped with 6 BlueRobotics T100 thrusters providing 5 degrees of freedom. The vehicle relied on three sensors namely, inertial measurement unit (IMU), depth sensor, and a camera for mapping and navigation. The team’s focus was to build a cost effective system while keeping in mind the performance. Most of the components and parts of the vehicle were fabricated and designed by the team members itself. 

Due to the outbreak of COVID-19, development of an AUV without having access to the hardware was a big challenge. With shift in the work environment from hostel rooms and college laboratories to Zoom and Google Meets, the pandemic pushed the team to fathom the deep waters of possibilities in search of a solution. This repository is the culmination of the work done by the team during the pandemic.

## References
- Kermorgant, Olivier. "A dynamic simulator for underwater vehicle-manipulators." International Conference on Simulation, Modeling, and Programming for Autonomous Robots. Springer, Cham, 2014.

[AUV Society]: https://auviiitdm.github.io/
[uwv_control]: ./uwv_control/README.md
[uwv_description]: ./uwv_description/README.md
[uwv_env]: ./uwv_env/README.md
[freefloating-gazebo]: https://github.com/freefloating-gazebo/freefloating_gazebo
[sauvc-simulations]: https://github.com/auvsocietyiiitdm/sauvc-simulations
[uwv-launcher]: https://github.com/auvsocietyiiitdm/uwv-launcher.git
