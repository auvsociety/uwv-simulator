# uwv_control/src/vec6/remote_control

## About
Contains source files to implement remote control mode on **vec6**. 

> Source files contributing to a ROS node for simulation are prefixed with `sim_` while those for hardware are prefixed with `hard_`.

## List of file names and use-cases

| File name | Use-case |
| --- | --- |
| `rc_config.h` | Contains macros aiding in defining steps and ROS node name.|
| `sim_remote_control.cpp` | A ROS node source file to control vec6 in ROV mode. |
| `sim_remote_control.h` | Header file for `sim_remote_control.cpp`. | 