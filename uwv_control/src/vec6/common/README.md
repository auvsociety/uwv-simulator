# uwv_control/src/vec6/common

## About
Contains common source files to build ROS nodes for the underwater vehicle: **vec6**.

## List of file names and use-cases

| File name | Use-case |
| --- | --- |
| `vec6_config.h` |  Contains constants, structures, and class (`Vec6State`) related to **vec6** vehicle. |
| `vec6_controller.cpp` | Abstract class to control the PID loop and configuration. |
| `vec6_comms.cpp` | Class to manage the publishing and subscribing of various topics. |
| `vec6_simcontroller.cpp` | Abstract class containing primitive functions aiding to build a ROS node capable of controlling the simulated **vec6** vehicle. |
| `vec6_hardwarecontroller.cpp` | Abstract class containing primitive functions aiding to build a ROS node capable of controlling the real **vec6** vehicle. |
| `terminal_getch.cpp` | Contains functions that allow getting character from user without echoing it. |