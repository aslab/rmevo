# RMEvo

The *Robot Morphology Evolutioner* is a toolkit oriented to explore and optimize hardware and software configurations for robots in certain environments. 

It uses Evolutionary Algorithms to evolve the morphology structure of a population of robots, represented in a tree structure. This project plays a part in the *Robominers project*, where it attempts to solve the search for optimal morphologies in self-reconfiguration.

## Project

### Main nodes

The RMEvo framework is composed by three main nodes:

- The RMEvo Core
  - Manage the population and applies the evolutionary algorithm.
  - Works with the genotype and the representation grammar.
  - Uses the fitness to perform crossover and mutation.
  - Applies selection.
- The Factory Node
  - Generates the SDF model using the available modules and configuration given by the Core.
  - Transforms genotype into phenotype.
  - Imports modules from specific folder and adds them to the available modules list.
- Gazebo world
  - Simulates a predefined world.
  - Evaluates the performance of the robot using predefined criteria and assigns a fitness.
- External nodes
  - The ROS structure allows external nodes to control the robot in the simulation

The relation between them is shown in the following image:
<p align="center">
  <img src="https://github.com/aslab/rmevo/blob/master/images/Nodes.svg" width="600">
</p>

### Centipede robot
The centipede robot is a basic modular robot designed as a test to show the main problems that the RMEvo has to solve.

It is a centipede-like robot formed by basic modules joined together. Its modularity allows the robot to change in shape, size and morphology.

The robot is defined in the package Centipede_description, a ROS package that contains the urdf models of the basic components of the robot.

A module composed of one body and two legs is shown behind:
<p align="center">
  <img src="https://github.com/aslab/rmevo/blob/master/images/centipede_module.png" width="600">
</p>

The instructions to build the repo and launch the basic world are the following:

```
# Create new folder and clone the repo
mkdir EvoRM
cd EvoRM
git clone https://github.com/aslab/rmevo.git

# Build the code and source it
catkin_make
source devel/setup.bash

# Launch the world with the basic model
roslaunch centipede_gazebo empty_world.launch
```


