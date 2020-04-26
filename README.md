# EvoRM

The **EvoRM** is a toolkit oriented to explore and optimize hardware and software configurations for the *Robominers Robot*. This project tackles the problem of self-reconfiguration in robots using evolutionary algorithms.

## Centipede_description
To test and apply the framework, a basic model has been developed. It is a centipede-like robot formed by basic modules joined together. That modularity allows the robot to change in shape, size and morphology.

Centipede_description is a ROS package that contains the urdf models of the basic components of the robot.

A module composed of one body and two legs is shown behind:
![Two-legged module](https://github.com/aslab/rmevo/blob/master/images/centipede_module.png)

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


