# uwv_env

## About
Contains files and models describing the environment. The [freefloating-gazebo] package is used to simulate the underwater conditions. This package also contains a script file that aids in constructing the environment.

Gazebo's [hydrodynamics plugin] is not opted for the simulation of underwater conditions because the plugin uses the bounding box around mesh of collision model to calculate the buoyant force. The results were accurate only when simple shapes like *cuboid*, *sphere*, and *cylinder* were used.

## Constructing the environment
TODO: Add instructions on how to use the script to create environment

<br/>

[Back to parent navigation](../README.md#navigate)


[freefloating-gazebo]: https://github.com/freefloating-gazebo/freefloating_gazebo
[hydrodynamics plugin]: http://gazebosim.org/tutorials?tut=hydrodynamics&cat=physics