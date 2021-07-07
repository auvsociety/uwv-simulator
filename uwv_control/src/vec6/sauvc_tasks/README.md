# uwv_control/src/vec6/sauvc_tasks

## About
The source files present in this directory generate a ROS node capable of performing the various SAUVC tasks.

> Source files contributing to a ROS node for simulation are prefixed with `sim_` while those for hardware are prefixed with `hard_`.

## List of file names and use-cases

| File name | Use-case |
| --- | --- |
| `tasks.cpp` | Contains classes that handle creation of tasks for the vehicle to perform and their execution. The classes defined here can be used for both simulation and for real hardware. |
| `tasks.h` | Header file for `tasks.cpp`. |
| `sim_task_list.cpp` | Contains classes that define the tasks to be executed for the simulation environment.  |
| `sim_task_list.h` | Header file for `sim_task_list.cpp`. |
| `sim_sampletasks_exe.cpp` | Contains an example implementation of usage and execution of tasks using the provided classes.  |
| `sim_sampletasks_exe.h` | Header file for `sim_sampletasks_exe.cpp`. |