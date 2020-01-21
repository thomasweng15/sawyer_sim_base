Sawyer Simulation Base
===

This repo provides a gazebo simulation of sawyer which can be used as a base for other projects.

# Installation

1. Create catkin workspace
2. Install this repo and the following dependencies into workspace src: intera_common, intera_sdk, sawyer_moveit, sawyer_robot, sawyer_simulator
3. Build the workspace

# Running the simulation
1. `roslaunch sawyer_sim_base simulation.launch`. You may be able to launch without the gazebo gui using `headless:=true gui:=false`.
2. `roslaunch sawyer_sim_base moveit.launch`


