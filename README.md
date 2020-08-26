# SUMO bot on ROS #

This repository tests whether one can set up a SUMO bot competition using ROS + Gazebo. The structure of this repo reflects how a SUMO bot would be built by general members of [UTRA](https://www.utra.ca/) (University of Toronto Robotics Association) [SUMO](https://www.utra.ca/teams/SUMO/).

The expected pipeline for completing a sumo bot is as follows:
1. Design CAD of robot (not done in this repository)
2. Spawn robot in Gazebo from CAD model
3. Add Gazebo plugins (motor, range/illuminance sensors)
4. Create a node to perform sumo competition algorithms
5. Compete!

Lastly, one would expect members to start from a template repository with packages they are not expected to create themselves (eg. `/gazebo_light_sensor_plugin` and `/worlds`).

## Overview of Packages ##

### /algorithm ###

Contains a node (written in either in C++ or Python) that subscribes to sensor topics, computes a command to move the robot and then publishes it to the appropriate topic.
> This node would contain all the logic for the desired algorithm.

### /description ###

Contains the code needed to simulate the sumo bot. A robot is built using URDF and the model (+ properties) from CAD. There are also launch files which serve to spawn Gazebo, RViz, the robot and start the *algorithm* node.
> Namespacing and ensuring appropriate simulation limits are respected need to be checked before allowing a sumo bot to compete!

### /gazebo_light_sensor_plugin ###

The code this package is adapted from is [this](https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/wiki/Sec.-4:-Creating-a-light-sensor-plugin) repository. It uses a small camera and computes illuminance by averaging pixel brightness. General members would not be expected to recreate this plugin from scratch, but it would be provided as part of a template repository.

### /worlds ###

A simple sumo ring consisting of a black circle with a radius of about 1 meter and white outside of the circle. To be able to use this world run:
```
chmod +x ./worlds/install_world.sh
```
> This installs the sumo_ring model (written in SDF) to the `~./gazebo/models` folder. General members would not recreate this plugin themselves, rather it would be part of a template repository.
