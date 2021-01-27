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

### Grammar

The genotype is represented using a tree structure. This allows to easily represent, visualize and work with the robot morphology.

Each node in the tree has the following attributes:

- Name: exclusive name for identification
- Type: the class in the available modules list from which the node inherits the non-variable properties
- Node parameters: variable parameters of the node such as color, size, or weights in a NN that can evolve with the population.
- Slots: array with references to the children of that node. The number of slots in a module comes predefined in the module template.

<p align="center">
  <img src="https://github.com/aslab/rmevo/blob/master/images/tree_node.svg" width="400">
</p>

### Module templates

The Factory node imports a folder containing the available modules. Those modules are presented in a SDF format, satisfying the SDF standard. Also, the modules have to include a special tag <rmevo>, where it is defined additional information for the RMEvo software. The number of slots and its position and orientation, for example, must be provided.

These templates are kept in the internal memory of the Factory node and are use to build the spawning robots.

### Centipede robot
The centipede robot is a basic modular robot designed as a test to show the main problems that the RMEvo has to solve.

It is a centipede-like robot formed by basic modules joined together. Its modularity allows the robot to change in shape, size and morphology.

The robot is defined in the package Centipede_description, a ROS package that contains the urdf models of the basic components of the robot.

A module composed of one body and two legs is shown behind:
<p align="center">
  <img src="https://raw.githubusercontent.com/aslab/rmevo/master/images/centipede_module.png" width="600">
</p>

### Wiki

If you want to know more, you can find the wiki [here](https://github.com/aslab/rmevo/wiki). It contains a general overview and some tutorials that will help you get through.